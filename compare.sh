#!/usr/bin/env bash
# compare.sh — compare ADS-B decoder outputs against readsb as truth source

BINFILE="data/modes1_2.4mhz.bin"
PYTHON=".venv/bin/python3"

TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

echo "=== Running decoders on $BINFILE ==="

# --- readsb (truth) ---
echo -n "readsb...         "
readsb --device-type ifile --ifile="$BINFILE" --iformat=UC8 --freq=1090000000 --raw 2>/dev/null \
    | tee "$TMP/readsb_raw.txt" \
    | grep -oP '(?<=\*)[0-9a-f]+(?=;)' \
    | sort -u > "$TMP/readsb.txt"
READSB_TOTAL=$(wc -l < "$TMP/readsb_raw.txt")
echo "$READSB_TOTAL raw / $(wc -l < "$TMP/readsb.txt") unique"

# --- basic_stream ---
echo -n "basic_stream...   "
$PYTHON basic_stream.py 2>/dev/null | tee "$TMP/basic_raw.txt" | \
    grep -oP '(?<=\*)[0-9a-f]+(?=;)' | sort -u > "$TMP/basic.txt"
BASIC_TOTAL=$(wc -l < "$TMP/basic_raw.txt")
echo "$BASIC_TOTAL raw / $(wc -l < "$TMP/basic.txt") unique"

# --- basic_stream copy ---
echo -n "basic_stream_copy... "
$PYTHON "basic_stream copy.py" 2>/dev/null | tee "$TMP/basic_copy_raw.txt" | \
    grep -oP '(?<=\*)[0-9a-f]+(?=;)' | sort -u > "$TMP/basic_copy.txt"
BASIC_COPY_TOTAL=$(wc -l < "$TMP/basic_copy_raw.txt")
echo "$BASIC_COPY_TOTAL raw / $(wc -l < "$TMP/basic_copy.txt") unique"

# --- adsb_24_simple ---
echo -n "adsb_24_simple... "
$PYTHON adsb_24_simple.py 2>/dev/null | tee "$TMP/simple_raw.txt" | \
    grep -oP '^[0-9a-f]+' | sort -u > "$TMP/simple.txt"
SIMPLE_TOTAL=$(wc -l < "$TMP/simple_raw.txt")
echo "$SIMPLE_TOTAL raw / $(wc -l < "$TMP/simple.txt") unique"

echo ""
echo "=== Unique message counts ==="
printf "  %-22s %s\n" "readsb (truth):"  "$(wc -l < "$TMP/readsb.txt")"
printf "  %-22s %s\n" "basic_stream:"       "$(wc -l < "$TMP/basic.txt")"
printf "  %-22s %s\n" "basic_stream_copy:"  "$(wc -l < "$TMP/basic_copy.txt")"
printf "  %-22s %s\n" "adsb_24_simple:"     "$(wc -l < "$TMP/simple.txt")"

show_diff() {
    local label="$1" truth="$2" decoder="$3"
    echo ""
    echo "=== $label vs readsb ==="
    local miss fp
    miss=$(comm -23 "$truth" "$decoder" | wc -l)
    fp=$(comm -13 "$truth" "$decoder" | wc -l)
    echo "  Missed  (in readsb, not $label): $miss"
    echo "  False + (in $label, not readsb): $fp"
    if [ "$miss" -gt 0 ]; then
        echo "  -- missed --"
        comm -23 "$truth" "$decoder" | sed 's/^/    /'
    fi
    if [ "$fp" -gt 0 ]; then
        echo "  -- false positives --"
        comm -13 "$truth" "$decoder" | sed 's/^/    /'
    fi
}

show_diff "basic_stream"      "$TMP/readsb.txt" "$TMP/basic.txt"
show_diff "basic_stream_copy" "$TMP/readsb.txt" "$TMP/basic_copy.txt"
show_diff "adsb_24_simple"    "$TMP/readsb.txt" "$TMP/simple.txt"

echo ""
echo "=== Done ==="
