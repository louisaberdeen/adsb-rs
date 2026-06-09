#!/usr/bin/env bash
# compare.sh — compare ADS-B decoder outputs against readsb as truth source
set -u

BINFILE="data/modes1_2.4mhz.bin"
PYTHON="${PYTHON:-.venv/bin/python3}"
RUST_BIN="adsb_rs/target/release/adsb_rs"

TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

echo "=== Running decoders on $BINFILE ==="

# --- readsb (truth) ---
# Use a live readsb if installed, otherwise fall back to the saved reference output
echo -n "readsb...         "
if command -v readsb >/dev/null 2>&1; then
    readsb --device-type ifile --ifile="$BINFILE" --iformat=UC8 --freq=1090000000 --raw 2>/dev/null \
        > "$TMP/readsb_raw.txt"
else
    echo -n "(using saved data/readsb_output.txt) "
    cp data/readsb_output.txt "$TMP/readsb_raw.txt"
fi
grep -oP '(?<=\*)[0-9a-f]+(?=;)' "$TMP/readsb_raw.txt" | sort -u > "$TMP/readsb.txt"
echo "$(wc -l < "$TMP/readsb_raw.txt") raw / $(wc -l < "$TMP/readsb.txt") unique"

# --- adsb_rs (Rust) ---
echo -n "adsb_rs...        "
if [ ! -x "$RUST_BIN" ]; then
    echo "building..."
    (cd adsb_rs && cargo build --release --quiet)
fi
"$RUST_BIN" "$BINFILE" 2>/dev/null | tee "$TMP/rust_raw.txt" | \
    grep -oP '^[0-9a-f]+' | sort -u > "$TMP/rust.txt"
echo "$(wc -l < "$TMP/rust_raw.txt") raw / $(wc -l < "$TMP/rust.txt") unique"

# --- adsb_24_simple (Python prototype) ---
echo -n "adsb_24_simple... "
if [ -x "$PYTHON" ]; then
    "$PYTHON" adsb_24_simple.py 2>/dev/null | tee "$TMP/simple_raw.txt" | \
        grep -oP '^[0-9a-f]+' | sort -u > "$TMP/simple.txt"
    echo "$(wc -l < "$TMP/simple_raw.txt") raw / $(wc -l < "$TMP/simple.txt") unique"
else
    echo "skipped ($PYTHON not found)"
    : > "$TMP/simple.txt"
fi

echo ""
echo "=== Unique message counts ==="
printf "  %-22s %s\n" "readsb (truth):"  "$(wc -l < "$TMP/readsb.txt")"
printf "  %-22s %s\n" "adsb_rs:"         "$(wc -l < "$TMP/rust.txt")"
printf "  %-22s %s\n" "adsb_24_simple:"  "$(wc -l < "$TMP/simple.txt")"

show_diff() {
    local label="$1" truth="$2" decoder="$3"
    [ -s "$decoder" ] || return 0
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

show_diff "adsb_rs"        "$TMP/readsb.txt" "$TMP/rust.txt"
show_diff "adsb_24_simple" "$TMP/readsb.txt" "$TMP/simple.txt"

echo ""
echo "=== Done ==="
