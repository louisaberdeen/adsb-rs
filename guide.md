# Building an ADS-B Decoder from Scratch
### A complete guide to `adsb_24_simple.py` — every line explained, every choice justified

---

## What ADS-B Actually Is

Every commercial aircraft in the sky is broadcasting its position right now. The system is called ADS-B (Automatic Dependent Surveillance–Broadcast), and it runs on 1090 MHz. Once per second or so, each transponder fires a 120-microsecond burst that contains the aircraft's ICAO address, altitude, position, velocity, and call sign, encoded in a protocol called Mode S. The protocol was designed in the 1990s and is beautifully simple — a single fixed carrier frequency, no spreading, no frequency hopping, just a raw 1 Mbit/s Manchester-coded pulse burst that any cheap radio can receive.

The RTL-SDR dongle that costs $25 online can receive it. What it gives you is a stream of raw IQ samples. Getting from there to decoded messages is the job of this decoder.

---

## Part 1: The Signal

### IQ Samples

The SDR doesn't give you audio. It gives you the raw complex baseband signal — pairs of numbers called I (in-phase) and Q (quadrature), sampled at the specified rate. Think of I and Q as the real and imaginary parts of a complex number representing the instantaneous state of the RF field. Together they encode both amplitude and phase.

For ADS-B, phase is noise. What we care about is **amplitude** — whether the transmitter is sending a pulse right now or not. The magnitude of the IQ pair is:

```
magnitude = sqrt(I² + Q²)
```

High magnitude means a pulse is present; low magnitude means silence.

### The UC8 Format

The binary file `modes1_2.4mhz.bin` is stored in UC8 format (Unsigned 8-bit I/Q), which is exactly what it sounds like: alternating 8-bit I and Q bytes, unsigned, with 127.5 representing zero (the midpoint of 0–255). This is the native format of RTL-SDR hardware.

Converting to magnitude in `file_streaming`:

```python
i_samples = (data[0::2].astype(np.float32) - 127.5) / 127.5   # I: 0–255 → -1.0..+1.0
q_samples = (data[1::2].astype(np.float32) - 127.5) / 127.5   # Q: same
magsq = np.minimum(i_samples**2 + q_samples**2, 1.0)           # clamp before sqrt
magnitude = (np.sqrt(magsq) * 65535.0 + 0.5).astype(np.uint16)
```

The clamp before the sqrt (`np.minimum(..., 1.0)`) handles the edge case where IQ values encode a magnitude slightly above 1.0 due to SDR gain settings. Without it you'd get values above 65535 which overflow uint16.

The result is a uint16 in range 0–65535. This matches the scale used by readsb internally, which matters because all the preamble correlation coefficients and threshold values are calibrated against this range. If you scale differently you'd need to rescale every magic number in the decoder.

### The RTL-SDR path

In `rtl_streaming`, `pyrtlsdr` delivers samples already as complex float32. The conversion is simpler — compute magnitude directly without the UC8 centering step. The scaling to uint16 is the same.

### Chunk Size and the Tail

The file is read in chunks of 64 KB (65,536 bytes = 32,768 IQ pairs = 32,768 magnitude samples). Why 64 KB? It's a conventional size that balances memory use and latency — small enough to flow through the async pipeline quickly, large enough that per-chunk overhead is negligible.

The critical detail: **ADS-B messages can straddle chunk boundaries**. The preamble might appear in the last few samples of one chunk, with the data in the next chunk. The solution is to keep the last 300 samples of every chunk (the "tail") and prepend it to the next chunk before scanning. 300 samples is enough because a full long message (preamble + 112 bits) spans at most 288 samples, with a little margin for phase variation.

---

## Part 2: The Preamble

Before any bits can be decoded, the decoder needs to find where a message starts. ADS-B messages begin with a specific 8-microsecond preamble pattern — four pulses at precise timing intervals — that acts as a synchronisation marker.

### The Preamble Structure

The preamble consists of four high-amplitude pulses:

```
Time (µs):  0    0.5   1.0   1.5   2.0   2.5   3.0   3.5   4.0   4.5   5.0
            |     |     |     |     |     |     |     |     |     |     |
Signal:     [HIGH][    LOW    ][HIGH][      LOW      ][HIGH][    LOW    ][HIGH][LOW]
```

More precisely: pulses at 0–0.5 µs, 1.0–1.5 µs, 3.5–4.0 µs, and 4.5–5.0 µs. The gaps between them are always low. The data payload follows immediately after, starting at 8 µs.

### Why 2.4 MHz Creates a Phase Problem

At exactly 2 MHz sampling you'd get 2 samples per 1-µs period — a clean integer ratio. But RTL-SDR hardware can't reliably produce exactly 2.0 MHz, and a slightly-off sample rate causes the symbol boundaries to drift through decoding. Readsb uses **2.4 MHz** instead: a deliberately non-integer rate.

At 2.4 MHz, each 500 ns symbol spans exactly **1.2 samples**. That's not an integer, which means no single sample straddles a symbol boundary the same way twice. After 5 symbols (5 × 1.2 = 6 samples), the pattern exactly repeats. There are therefore exactly **5 distinct sub-sample phases** (0 through 4) — telling you where within a sample the symbol boundary falls. The decoder tracks this phase throughout the message.

The trade-off: instead of one "aligned" decoding, you need five. The benefit: perfect stability — no timing drift, no symbol boundary ambiguity, maximum SNR from matched filtering.

### The Quick Pre-Check

Checking all 5 phases at every sample position would be extremely slow. The first line of defence is a cheap three-comparison pre-check derived from the preamble shape:

```python
if not (samples[i+1] > samples[i+7] and
        samples[i+12] > samples[i+14] and
        samples[i+12] > samples[i+15]):
    continue
```

This checks specific sample positions that should be high vs low in any valid preamble, regardless of phase. Position 1 is during the first pulse; position 7 is in the first gap; positions 12 and 13 are during the third pulse; 14 and 15 are in the gap before the fourth pulse. If these basic relationships don't hold, we skip without computing anything expensive. In practice this eliminates ~99% of positions.

### Adaptive Noise Threshold

Once a position passes the quick check, we measure the local noise floor and set a threshold:

```python
noise = (int(samples[i+5]) + int(samples[i+8]) +
         int(samples[i+16]) + int(samples[i+17]) + int(samples[i+18]))
ref_level = (noise * threshold_factor) >> 5   # = noise * factor / 32
```

Samples at positions 5, 8, 16, 17, 18 are within the preamble's mandatory quiet gaps — they're always low regardless of phase. Their sum estimates the local noise floor. The threshold is then `noise × factor / 32`. Division by 32 (a power of two) lets this be a cheap right-shift.

The `threshold_factor` is empirically tuned. Value 50 gives `50/32 ≈ 1.56×` the noise floor. Try too low and noise triggers false detections everywhere. Try too high and weak real signals get rejected. The readsb default is 58 (`58/32 ≈ 1.81×`). For this decoder, 50 was found to be optimal in testing — more sensitive without meaningfully more false positives. This is the kind of value you tune against real data, not derive from first principles.

### Phase-Aware Preamble Correlation

After passing the threshold check, we compute the preamble correlation magnitude for each of the 5 phases:

```python
def compute_preamble_magnitude(samples, i, phase):
```

The preamble is a known pattern. We know exactly which samples should be high (pulse present) and which should be low (gap). The correlation is essentially a dot product between the received samples and the expected pattern. Readsb provides the expected weights for each phase:

```
phase 0: weights = [2, 4, 0, 5, 1, 0, 0, 0, 0, 5, 1, 3, 3, 0, ...]
phase 1: weights = [1, 5, 0, 4, 2, 0, 0, 0, 0, 4, 2, 2, 4, 0, ...]
...
```

Computing the full dot product for all 5 phases at every candidate position would cost 100 multiplications. The algebraic simplification used here (and in readsb) collapses them:

```python
diff_2_3  = int(pa[2]) - int(pa[3])
sum_1_4   = int(pa[1]) + int(pa[4])
diff_10_11 = int(pa[10]) - int(pa[11])
common3456 = sum_1_4 - diff_2_3 + int(pa[9]) + int(pa[12])

# Then branch on phase for the differing term:
pa_mag = common3456 - diff_10_11  # phases 0 and 1
pa_mag = common3456 + diff_10_11  # phases 2 and 3
pa_mag = sum_1_4 + 2*diff_2_3 + diff_10_11 + pa[12]  # phase 4
```

The terms that are identical across phases are computed once (`common3456`), then the phase-specific term is added. This is about 10 additions instead of 100 multiplications.

The phase with the highest magnitude above `ref_level` wins. We save it as `best_phase` for the decoder.

---

## Part 3: Deduplication — The Blind Skip Problem and NMS

### Why You Need Deduplication

A real preamble at position N will pass the correlation check not just at N but also at N-1, N+1, N+2, etc. The preamble is 8 µs wide and the quick-check comparison holds true across a range of positions. Without deduplication, one physical transmission generates 5–20 candidates queued to the decoder, burning CPU on redundant work.

The naive solution is a blind skip: after emitting a candidate at position N, advance by `preamble_skip` samples before looking again. Simple, but deeply flawed.

### The Problem with Blind Skip

Consider this scenario:
- A weak noise spike passes the threshold at position N (false positive).
- The decoder skips forward by 20 samples.
- A real strong preamble starts at position N+5.
- It falls within the skip window — missed.

In testing, blind skip with `N=20` missed 58 of 168 messages compared to readsb truth. The false positives aren't just annoyances; they actively destroy real signal detection.

### Non-Maximum Suppression (NMS)

The fix is NMS. Instead of skipping blindly, track a "window" of recent candidates and only emit the one with the highest correlation magnitude:

```python
win_i = None      # position of the current window's best candidate
win_mag = 0
win_phase = -1

if best_phase >= 0:
    if win_i is None:
        win_i, win_mag, win_phase = i, best_mag, best_phase
    elif i - win_i <= nms_window:
        if best_mag > win_mag:          # replace if this one is stronger
            win_i, win_mag, win_phase = i, best_mag, best_phase
    else:
        # moved past the window — emit the winner, start a new window
        await out_queue.put(ADSBCandidate(samples[win_i:win_i+300], win_phase, count))
        win_i, win_mag, win_phase = i, best_mag, best_phase
```

At end of each chunk, flush the current window's winner. Candidates near the chunk boundary may be re-detected in the next chunk (via `samples_tail`), but the decoder's ICAO filter handles duplicates at that stage.

The key insight: a false positive and a real preamble within 5 samples of each other will always produce different magnitudes — the real one is stronger. NMS picks the stronger one automatically.

In testing, NMS with window=5 recovered 165 of 168 messages — matching `basic_stream.py`'s 166. The difference drops from 58 missed to 3 missed. The remaining 3 missed messages appear to be genuine weak signals that neither decoder recovers reliably.

---

## Part 4: Decoding

### Manchester Encoding

ADS-B uses Manchester encoding for the data payload. Each bit is two symbols:
- `1` = high then low
- `0` = low then high

Each symbol is 500 ns. At 2.4 MHz sampling (1 sample per 416.67 ns), one symbol spans exactly 1.2 samples. One **bit** therefore spans exactly **2.4 samples**.

### The Phase Offset Between Preamble and Data

The preamble detector locks onto the start of the preamble (sample position 0). The data payload begins 8 µs = 19.2 samples later. The phase detected in the preamble is measured at the preamble's first pulse. The relationship between this "preamble phase" and the sub-sample phase of the first data bit involves a fixed offset of −1 (mod 5):

```python
data_phase = (preamble_phase + 4) % 5   # +4 ≡ -1 mod 5
```

This was established empirically by cross-referencing with readsb's `score_phase()` logic. The preamble correlator and the data correlator use slightly different reference points within the sample window, producing this constant −1 offset.

There's also a special case for phase 0: the data starts at sample 19 instead of 20:

```python
preamble_samples = 19 if trial_phase == 0 else 20
```

Phase 0 is the "early" phase where the signal arrives 0 sub-samples into the sample period. The preamble pointer therefore ends up one sample earlier, pushing the data start back by one. This matches readsb's `pa += 19` vs `pa += 20` in `scorePhase()`.

### Integer Arithmetic for Sample Positions

The obvious way to track the current sample position while decoding:

```python
frac_pos = 0.0
frac_pos += 2.4   # per bit
pos = start + int(frac_pos)
```

This accumulates floating-point rounding errors. By bit 112, `frac_pos` might be off by 0.1–0.2 of a sample, potentially reading from the wrong position.

The exact integer formulation avoids this entirely. For bit `n` starting at `data_phase p`:

```
sample_offset(n) = (12*n + p) // 5
```

Why does this work? 2.4 = 12/5. Bit `n` starts at exactly `(12n/5)` samples from the data start, offset by `p/5` for the initial phase. Integer floor division gives the exact integer sample position without any accumulated error. In Python:

```python
sample_offset = (12 * bit_idx + data_phase) // 5
subsample_phase = (2 * bit_idx + data_phase) % 5
```

The `subsample_phase` advances by 2 per bit (one bit = two symbols = two phase steps of 1 each) and wraps mod 5. This is the exact phase at which to apply the correlation kernel for that bit.

### The Correlation Kernels

Each bit's Manchester pulse is decoded by correlating a short window of samples against the expected pulse shape for the current sub-sample phase:

```
phase 0:  18*s0 - 15*s1 -  3*s2          (positive → bit=1, negative → bit=0)
phase 1:  14*s0 -  5*s1 -  9*s2
phase 2:  16*s0 +  5*s1 - 20*s2
phase 3:   7*s0 + 11*s1 - 18*s2
phase 4:   4*s0 + 15*s1 - 20*s2 + s3     (needs a 4th sample)
```

The intuition: at phase 0, the HIGH symbol aligns mostly with sample 0 (large positive weight) and the LOW symbol aligns mostly with sample 1 (large negative weight). At phase 4, the transition falls between samples 1 and 2, so sample 1 gets the largest weight. The coefficients are pre-computed approximations to the overlap integrals — what fraction of each sample falls within the HIGH vs LOW symbol.

Phase 4 requires a 4th sample because its HIGH symbol straddles the boundary between samples 1 and 2, and the LOW symbol extends into sample 3. All other phases fit within 3 samples.

### Downlink Format Codes

The first 5 bits of every Mode S message encode the **Downlink Format (DF)**, which determines the message type and length. Not all values are valid:

```python
VALID_DF_SHORT = {0, 4, 5, 11}        # 56-bit messages
VALID_DF_LONG  = {16, 17, 18, 19, 20, 21, 24}   # 112-bit messages
```

If the first 5 bits decode to anything else, this phase is wrong — return None immediately without decoding the rest. This is a cheap filter that eliminates a large fraction of bad phase guesses before the expensive CRC.

The message length is determined by the MSB of the DF: if bit 0 is 1 (DF ≥ 16), the message is 112 bits; otherwise 56. This is a simplification that holds for all current DF codes. DF 19 in `VALID_DF_LONG` is included in this decoder (it was absent from early versions, causing missed messages).

### The CRC

Mode S uses a 24-bit CRC with generator polynomial x²⁴ + x²³ + x¹⁰ + x³ + 1, represented as `0xFFF409`. The last 3 bytes of every message hold the transmitted CRC.

```python
def modes_checksum(msg):
    POLY = 0xFFF409
    crc = 0
    for byte in msg[:-3]:
        crc ^= (byte << 16)   # XOR byte into top 8 bits of 24-bit register
        for _ in range(8):
            if crc & 0x800000:
                crc = (crc << 1) ^ POLY
            else:
                crc <<= 1
            crc &= 0xFFFFFF
    crc ^= (msg[-3] << 16) | (msg[-2] << 8) | msg[-1]
    return crc
```

For message types that include the aircraft's ICAO address in the payload (DF 17, 18 — the "Extended Squitter" formats), a valid message has `crc == 0`. This is an enormously strong filter: a random 112-bit string has a 1-in-16-million chance of passing by luck.

### The ICAO Filter

For **other** message types (DF 0, 4, 5, 11, 16, 20, 21, 24), the CRC isn't transmitted in isolation — the CRC residual *is* the ICAO address of the sending aircraft. The receiver can verify these by checking whether the CRC residual matches a known ICAO:

```python
if crc not in icao_filter:
    continue
```

The `icao_filter` is a set of ICAO addresses that have been confirmed through prior DF 17/18 messages. DF 17/18 are "self-identifying" (ICAO is plaintext in the payload) and CRC-clean. They are accepted unconditionally and their ICAO added to the filter. Subsequent non-self-identifying messages from the same aircraft are then accepted by residual matching.

DF 11 (All-Call Reply) is semi-self-identifying: the top 17 bits of the CRC residual must be zero (only the bottom 7 bits carry an Interrogator ID). It's accepted if `crc & 0xFFFF80 == 0`, and its ICAO is extracted and added to the filter.

This design means the decoder can only recover DF 0/4/5 etc. from aircraft it has already "seen" via DF 17/18 or DF 11. In a short capture like the test file (178 ms), this limits the count — but in a long real-time session it works very well.

### Scoring

When multiple phase trials produce valid messages, a scoring system picks the best:

```python
score = 1800   # DF 17/18 from a previously-seen ICAO
score = 1400   # DF 17/18 new aircraft
score = 1000   # DF 11
score = 800    # DF 0/4/5/16/20/21/24 via ICAO residual
```

In practice, the first phase that passes CRC is accepted immediately (`break` after the first success). The scoring system exists to handle the hypothetical case where two phases both produce valid messages, and is a direct port of readsb's priority ordering.

### Spiral Phase Iteration

The preamble correlator found a `best_phase` — the phase with the highest preamble correlation score. There's a reasonably strong correlation between preamble phase and decode phase in practice, but not a perfect one. Rather than trying all 5 phases in order 0–4, the decoder uses the preamble's estimate as the starting point and expands outward:

```python
def spiral_from(start, n):
    result = [start]
    for delta in range(1, n):
        if start - delta >= 0:
            result.append(start - delta)
        if start + delta < n:
            result.append(start + delta)
    return result

# spiral_from(2, 5) → [2, 1, 3, 0, 4]
```

This is non-modular (no wrap-around) because phases 0 and 4 are not equivalent — they represent physically different sub-sample alignments. The most likely correct phase is tried first, saving decoder work on the common case.

---

## Part 5: The Pipeline and Configuration

### Async Pipeline

The decoder is structured as three async coroutines connected by queues:

```
file_streaming → [raw_q] → preamble_detection_nms → [candidate_q] → decoding
```

Each stage runs concurrently. Python's GIL means there's no true parallelism for CPU-bound code, but for a file-based decoder this doesn't matter — the bottleneck is pure CPU work in the inner loops, and asyncio's cooperative scheduling keeps the stages balanced.

The `EndOfStream` sentinel dataclass signals pipeline completion cleanly:

```python
@dataclass
class EndOfStream:
    pass
```

Each stage reads from its input queue and pattern-matches on the type:

```python
match item:
    case EndOfStream():
        await out_queue.put(EndOfStream())
        break
    case SampleChunk() as chunk:
        ...
```

This is Python's structural pattern matching (3.10+), used here as the equivalent of Rust's `match` on `Option`/`Result` types. When porting to Rust, `EndOfStream` maps naturally to `None` on a channel close or an `Option<Chunk>` in the message type.

### Configuration

```python
@dataclass
class ADSBConfig:
    sample_rate: float = 2.4e6
    center_freq: float = 1090e6
    gain: str = 'auto'
    chunk_size: int = 64*1024
    debug: bool = False
    realtime: bool = True
    threshold_factor: int = 50    # SNR threshold: noise * factor / 32
    preamble_skip: int = 5        # NMS window size in samples
    use_nms: bool = True          # NMS (True) or blind skip (False)
```

`threshold_factor = 50`: tested against readsb truth; higher values (52, 58) reduce sensitivity without reducing false positives enough to compensate.

`preamble_skip = 5`: the NMS window. Tested at 5, 10, 15, 20 — window=5 gives 165/168 messages vs readsb; window=20 gives only 156/168. The window is a minimum separation between emitted candidates, so smaller is better as long as it prevents the immediate duplicate at position N±1.

---

## Part 6: What Didn't Work (and Why)

### Blind Skip Was Fundamentally Wrong

The first implementation used `i += preamble_skip` after every candidate detection. The logic seemed sound: skip past the current message before looking for the next one. The problem is that the skip fires on **every** candidate, including false positives — and false positives fire precisely at high-signal areas where real messages are also likely. With skip=20, we missed 58 of 168 messages.

### Vectorized Preamble Detection

A numpy vectorized version was implemented that computed the quick-check conditions for all positions simultaneously using array indexing, then ran preamble correlation on the surviving positions. This was fast (fewer Python loop iterations) and got 166/168 messages with a proper NMS implementation.

It was ultimately removed because: (a) the scalar version with NMS is nearly as accurate (165/168), (b) adding numpy vectorization to a Rust port would mean SIMD intrinsics where LLVM auto-vectorization handles it better, and (c) the scalar version is much easier to read and port.

### Vectorized Bit Decoding

A fully vectorized decoder was implemented that packed all 112 bit positions and 5 phases into numpy array operations. It was slower than scalar, not faster — the overhead of constructing numpy slices for 112-element arrays outweighed the computation saved. Vectorization wins when operating on thousands of elements at once; for decoding a single 112-bit message it loses badly. The fix in Rust is trivial: LLVM auto-vectorizes the inner loops automatically.

### Floating-Point Position Accumulation

An early version of `decode_bits` accumulated position as `frac_pos += 2.4`, taking `int(frac_pos)` each iteration. This worked for most messages but produced occasional off-by-one errors on bits 40–112 where accumulated float rounding (~0.15 samples of error by bit 100) occasionally hit a sample boundary. The fix was the `(12*n + p) // 5` integer formula, which is exact.

### threshold_factor = 52

The original code used 52 (the readsb default in some configurations). Testing showed that 50 recovered more messages without a meaningful increase in false positives. The threshold is a genuine free parameter that should be tuned for your deployment environment — high-gain antenna in a clear area can afford a stricter threshold; indoor or obstructed setups benefit from a lower one.

### DF 19 Missing

Early versions of `VALID_DF_LONG` didn't include DF 19 (Military Extended Squitter). Adding it recovered no additional messages on this test file (no military traffic in the capture), but it's correct to include it for completeness.

---

## Appendix: Numbers to Know

| Quantity | Value |
|---|---|
| Center frequency | 1090 MHz |
| Sample rate | 2.4 MHz |
| Sample period | 416.67 ns |
| Symbol period | 500 ns |
| Samples per symbol | 1.2 |
| Sub-sample phases | 5 |
| Bits per phase cycle | 5 |
| Samples per phase cycle | 12 |
| Preamble duration | 8 µs = 19.2 samples |
| Short message | 56 bits = 7 bytes |
| Long message | 112 bits = 14 bytes |
| Short message span | ~135 samples |
| Long message span | ~269 samples |
| Candidate window | 300 samples |
| Chunk size | 32,768 magnitude samples |
| Test file duration | 178 ms |
| CRC polynomial | 0xFFF409 |
| CRC length | 24 bits |

---

## Setup and Useful Commands

```bash
# Check RTL-SDR is recognised
rtl_test

# Stop readsb if it's holding the device
sudo systemctl stop readsb
sudo systemctl restart readsb

# Check nothing else is using the SDR
ps aux | grep -E 'rtl|sdr'
```

### Getting Test Data

The test file `modes1_2.4mhz.bin` was derived from dump1090's sample data resampled to 2.4 MHz:

```bash
# Download dump1090 test file (2 MHz, UC8 format)
git clone --depth 1 https://github.com/antirez/dump1090.git /tmp/dump1090
cp /tmp/dump1090/testfiles/modes1.bin data/

# Resample from 2 MHz → 2.4 MHz (required for this decoder and readsb)
python3 data/resample_iq.py data/modes1.bin data/modes1_2.4mhz.bin --from-rate 2000000 --to-rate 2400000

# Verify with readsb (truth source)
readsb --device-type ifile --ifile data/modes1_2.4mhz.bin --iformat UC8 --raw 2>&1 | head -20

# Verify original 2 MHz file with dump1090
/tmp/dump1090/dump1090 --ifile data/modes1.bin --raw 2>&1 | head -20
```

---

## Quick Reference: Porting to Rust

The Python pipeline maps cleanly to tokio channels:

```
file_streaming task → mpsc::channel<Chunk> → preamble task → mpsc::channel<Candidate> → decode task
```

`EndOfStream` maps to channel close (dropping the sender). Pattern match on `Option<Chunk>`.

The inner loops are pure scalar arithmetic — no numpy needed. LLVM will auto-vectorize them. Use `u16` for magnitudes (matches the uint16 range), `i32` for correlations (products of u16 * i8-scale coefficients fit comfortably).

The `icao_filter` accumulates across decoded messages. If you parallelize decoding across candidates with rayon, either keep the filter on the main task (check and update after each candidate result arrives) or use `DashMap<u32, ()>` for concurrent access without a mutex.

`spiral_from` is a no-alloc iterator in Rust — implement it as a struct with `Iterator` that yields the same sequence.

The `modes_checksum` inner loop is 8 iterations per byte, 11–14 bytes per message — around 100 iterations total. In Rust this is trivially fast; no CRC crate needed.

---

## Part 7: From Hex to Aircraft Map — Parsing DF17 Payloads

Up to this point the decoder produces a stream of validated hex strings. That's useful for raw monitoring, but a map needs to know *where* an aircraft is, *how fast* it's going, and *what it's called*. All of that information is in the ME field of DF17 Extended Squitter messages, encoded in a compact binary format defined by ICAO Annex 10. This part walks through how each field is extracted and why the format is designed the way it is.

---

### The DF17 Message Structure

A 112-bit (14-byte) DF17 message is laid out as:

```
Byte  0       1       2       3       4       5       6       7       8       9      10      11      12      13
      [DF+CA ][  ICAO address (3 bytes)              ][  ME field (7 bytes)                        ][  CRC-24 (3 bytes)     ]
      76543210
      DF = bits 7-3 (5 bits) = 17 for Extended Squitter
      CA = bits 2-0 (3 bits) = capability
```

- **DF (Downlink Format, 5 bits):** always 17 for ADS-B. This is the value the decoder checked when deciding message length.
- **CA (Capability, 3 bits):** flight status and interrogator capability. Not decoded by this project.
- **ICAO (24 bits, bytes 1–3):** the unique aircraft address, the same one the ICAO filter has already extracted.
- **ME (56 bits, bytes 4–10):** the payload. Everything interesting lives here.
- **CRC-24 (bytes 11–13):** must equal zero for a valid DF17 — already verified before this point.

The first 5 bits of the ME field are the **Type Code (TC)**, which determines what the remaining 51 bits contain:

| TC | Content |
|---|---|
| 1–4 | Aircraft identification (callsign) |
| 5–8 | Surface position |
| 9–18 | Airborne position with barometric altitude |
| 19 | Airborne velocity |
| 20–22 | Airborne position with GNSS altitude |

This decoder handles TC 1–4, 9–18, and 19 — callsign, baro position, and velocity. Surface position and GNSS altitude are structurally similar but not implemented.

---

### TC 1–4: Callsign

The callsign is 8 characters packed at 6 bits each into the 48 bits following the Type Code (50 bits total with the 5-bit TC, leaving 1 bit unused but that's fine). Why 6 bits per character? The AIS (Aeronautical Information Services) character set has exactly 64 members — A-Z (26), 0-9 (10), space, and some punctuation — which fits in 6 bits.

The 6-bit encoding maps:

```
1–26  →  A–Z
32    →  space
48–57 →  0–9
```

Value 0 is not a valid character; values 27–31 and 33–47 are technically reserved. In practice, the only characters you'll see in an ICAO callsign are A–Z, 0–9, and trailing spaces.

Extracting the characters from the ME bytes:

```rust
// ME bytes are indexed from byte 0 of the ME field
// TC occupies bits 0-4. Characters start at bit 5.
let chars: Vec<char> = (0..8).map(|i| {
    // Bit offset of this character from the start of ME
    let bit_start = 5 + i * 6;
    let byte_idx  = bit_start / 8;
    let bit_off   = bit_start % 8;
    // Extract 6 bits, potentially spanning two bytes
    let val = ((me_u64 >> (48 - bit_start)) & 0x3F) as u8;
    match val {
        1..=26 => (b'A' + val - 1) as char,
        48..=57 => (b'0' + val - 48) as char,
        32      => ' ',
        _       => '?',
    }
}).collect();
```

The trick with `me_u64`: packing all 7 ME bytes into a single `u64` (with the MSB zero) means any 6-bit field can be extracted by a single right-shift and mask without worrying about byte boundaries. This is the same trick used for altitude and CPR extraction — it's cleaner than manual byte indexing.

Callsigns are 8 characters, almost always padded with trailing spaces (`"RYR1234 "`). The state tracker trims them before storing (`trim_end()`), since tar1090 displays them with a trailing space in its tooltip anyway.

---

### TC 9–18: Airborne Position

This type code encodes two completely different things in the same 7-byte field: altitude and position. The position encoding (CPR) is detailed enough to deserve its own section.

#### The ME Field Layout for TC 9–18

```
Bit offsets within ME (0 = MSB of ME byte 0):
  0– 4:  Type Code TC (5 bits)
  5– 6:  Surveillance status (2 bits, ignored)
  7:     Single antenna flag (1 bit, ignored)
  8–19:  Altitude (12 bits)
  20:    Time synchronisation flag (1 bit, ignored)
  21:    F-flag — 0 = even frame, 1 = odd frame (1 bit)
  22–38: CPR latitude (17 bits)
  39–55: CPR longitude (17 bits)
```

Total: 56 bits = 7 bytes. The bit numbering is big-endian; bit 0 is the most significant.

Packing all 7 ME bytes into a `u64` and extracting by absolute bit position avoids any ambiguity about which byte a field lives in:

```rust
let me_u64 = u64::from_be_bytes([0, me[0], me[1], me[2], me[3], me[4], me[5], me[6]]);

// Altitude occupies bits 8-19 from the top → shift right by (55-19)=36, mask 12 bits
let alt_raw = ((me_u64 >> 36) & 0xFFF) as u32;

// F-flag is bit 21 → shift right by (55-21)=34
let f_bit = ((me_u64 >> 34) & 1) != 0;

// CPR latitude occupies bits 22-38 → shift right by (55-38)=17, mask 17 bits
let lat17 = ((me_u64 >> 17) & 0x1FFFF) as u32;

// CPR longitude occupies bits 39-55 → no shift needed, mask 17 bits
let lon17 = (me_u64 & 0x1FFFF) as u32;
```

The right-shift amounts come directly from the bit layout: to extract a field whose MSB is at bit position `p` (0 = top of the 56-bit ME field), the u64 has 8 leading zero bits, so the shift is `(63 - p - field_width + 1)` = `(56 - p - 1)` for the MSB, or equivalently shift by `(55 - LSB_position)`. Counting from the bottom: longitude MSB is at position 55 − 39 = 16, so mask 17 bits starting at bit 16 → `>> 0 & 0x1FFFF`. Latitude MSB at 55 − 22 = 33, so `>> 17 & 0x1FFFF`. This is the mechanical translation from bit-layout diagram to Rust shifts.

#### Altitude Decoding

The 12-bit altitude field uses the Q-bit (bit 4 within the 12-bit field, counting from LSB) to select between two encodings:

**Q-bit = 1 (the common case):** The remaining 11 bits (5 high bits + 6 low bits around the Q-bit) encode altitude directly:

```rust
let q_bit = (alt_raw >> 4) & 1;
if q_bit == 1 {
    // Remove the Q-bit: bits above it shift right by 1
    let n = ((alt_raw & 0xFE0) >> 1) | (alt_raw & 0x00F);
    // n=0 means "altitude not available"
    if n == 0 { return None; }
    altitude_feet = n as i32 * 25 - 1000
}
```

The formula `n * 25 - 1000` gives altitude in feet MSL. The − 1000 offset means the encoding can represent altitudes below sea level (n=1 → −975 ft). The 25-foot resolution is coarser than GPS but fine for ATC purposes.

**Q-bit = 0:** Gillham encoding (a Gray-code variant). Used at very high altitudes and in older transponders. This decoder returns `None` for Q-bit=0 — it's uncommon and the Gillham decode adds significant complexity for minimal benefit.

---

### CPR: Compact Position Reporting

CPR is the most algorithmically interesting part of ADS-B decoding. The 17 CPR latitude bits and 17 CPR longitude bits are **not** simply a scaled latitude and longitude — they encode a fractional position within a *zone*, and the zone identity is ambiguous from a single frame. Two consecutive frames (one even, one odd) together resolve the ambiguity and give a precise global position.

#### Why Two Frames?

17 bits can encode 2¹⁷ = 131,072 distinct values, giving a resolution of 360°/131,072 ≈ 0.00274° ≈ 0.3 km per tick — fine enough. But to avoid transmitting two separate 17-bit numbers (lat + lon), CPR encodes position as a fraction within a latitude zone, and alternates between two different zone sizes. The zone size difference is what breaks the ambiguity.

Concretely:
- **Even frames** (F=0): divide the globe into 60 latitude bands of 6° each.
- **Odd frames** (F=1): divide the globe into 59 latitude bands of ≈ 6.10° each.

A single frame tells you *where within a zone* the aircraft is, but not *which zone*. Two frames with different zone counts let you solve for both the zone and the position within it. The 60-vs-59 difference means the system of equations has a unique solution within a region small enough to be unambiguous in practice (within about 360° / gcd(60,59) = 360° — so theoretically not unique globally, but the aircraft's position from its first fix resolves the remaining ambiguity).

#### The Global Decode Algorithm

Given a recent even frame `(lat_e, lon_e)` and odd frame `(lat_o, lon_o)`, all as 17-bit unsigned integers in [0, 131071]:

**Step 1 — Latitude zone index:**

```
j = floor(59 × lat_e/131072 − 60 × lat_o/131072 + 0.5)
```

This estimates which zone pair the aircraft is in. The floor-plus-half is standard rounding to the nearest integer.

**Step 2 — Decoded latitudes:**

```rust
let dlat_e = 360.0 / 60.0;   // = 6.0°
let dlat_o = 360.0 / 59.0;   // ≈ 6.1017°

let lat_e_deg = dlat_e * (j.rem_euclid(60) as f64 + lat_e as f64 / 131072.0);
let lat_o_deg = dlat_o * (j.rem_euclid(59) as f64 + lat_o as f64 / 131072.0);
```

The `rem_euclid(60/59)` keeps the zone index in-range regardless of the sign of `j`. The fractional part `lat_e / 131072.0` is the aircraft's position *within* that zone.

**Step 3 — Latitude consistency check:**

The two computed latitudes should agree closely. If `|lat_e_deg − lat_o_deg| > 3°` (half a zone), the frames are from different zones and the decode is invalid. Discard and wait for the next pair.

**Step 4 — Select latitude by frame age:**

Use whichever frame arrived more recently. If the even frame is newer, use `lat_e_deg`; if odd is newer, use `lat_o_deg`.

**Step 5 — Longitude zone count:**

```rust
let nl_e = nl(lat_e_deg);
let nl_o = nl(lat_o_deg);
if nl_e != nl_o { return None; }   // frames span a zone boundary — discard
```

`nl(lat)` returns the number of longitude zones at a given latitude, ranging from 59 at the equator to 1 at the poles. It uses the ICAO Annex 10 Appendix B table — a series of latitude thresholds, each marking where the zone count decreases by one as you move towards the pole. The table has 58 entries; see the Bug Record section at the end of this document for what happens when the table is wrong.

**Step 6 — Longitude zone index:**

```
m = floor(lon_e/131072 × (nl_e − 1) − lon_o/131072 × nl_e + 0.5)
```

Same structure as Step 1 but for longitude.

**Step 7 — Decoded longitude:**

```rust
let ni = if even_newer { nl_e.max(1) } else { (nl_o - 1).max(1) };
let dlon = 360.0 / ni as f64;
let lon_deg = dlon * (m.rem_euclid(ni as i64) as f64 + lon_ref as f64 / 131072.0);
```

Where `lon_ref` is the CPR longitude from whichever frame is newer.

**Step 8 — Normalise:**

Wrap latitude into [−90, 90] and longitude into [−180, 180]. Subtract 360° if the value exceeds half the range.

#### The 10-Second Window

A position is only decoded if the even and odd frames are less than 10 seconds apart. If the aircraft is manoeuvring, older frames from a different location would produce a nonsense position. The 10-second limit is conservative (at 900 km/h a plane moves only ~2.5 km in 10 seconds, well within the ~180 km zone width at mid-latitudes) but generous enough that a brief signal dropout doesn't force a cold start.

---

### TC 19: Velocity

Velocity is the simplest decode — no zone arithmetic, just direct measurement.

The ME field for TC 19, Subtype 1 or 2 (ground speed, the common case):

```
Bits  5– 7:  Subtype (3 bits), 1 = subsonic, 2 = supersonic
Bit   8:     Intent change flag
Bit   9:     IFR capability flag
Bits 10–12:  NACv (navigation accuracy, ignored)
Bit  13:     East-West direction (0 = east, 1 = west)
Bits 14–23:  East-West speed (10 bits, knots+1; 0 = not available)
Bit  24:     North-South direction (0 = north, 1 = south)
Bits 25–34:  North-South speed (10 bits, knots+1; 0 = not available)
Bit  35:     Vertical rate source (0 = baro, 1 = GNSS)
Bit  36:     Vertical rate sign (0 = up, 1 = down)
Bits 37–45:  Vertical rate magnitude (9 bits, 64 fpm increments + 1)
Bits 46–47:  Reserved
Bit  48:     GNSS/baro altitude difference sign
Bits 49–55:  GNSS/baro altitude difference (7 bits, 25 ft increments)
```

The `knots + 1` encoding means a value of 1 = 0 kt, 2 = 1 kt, etc., and 0 is the "not available" sentinel. Subtract 1 before using.

Converting EW/NS components to ground speed and track:

```rust
let vew = if ew_dir { -(ew_spd as f64) } else { ew_spd as f64 };  // west = negative
let vns = if ns_dir { -(ns_spd as f64) } else { ns_spd as f64 };  // south = negative

let gs    = (vew * vew + vns * vns).sqrt();                         // knots
let track = vew.atan2(vns).to_degrees().rem_euclid(360.0);         // 0 = north, clockwise
```

`atan2(vew, vns)` with the E-W component as the *first* argument is the correct convention for bearing (as opposed to standard mathematical angle, which measures from the east axis). `rem_euclid(360.0)` normalises negative angles.

Vertical rate: `(vr_raw - 1) * 64` feet per minute, negated if the sign bit is set. The `- 1` removes the same availability sentinel offset.

---

### The State Tracker

Individual messages are snapshots. A callsign comes in one message, a position in the next, velocity in another. The state tracker is the glue — it maintains a `HashMap<u32, AircraftState>` keyed by ICAO address and merges each new message into the appropriate record.

```rust
struct AircraftState {
    callsign:     Option<String>,
    alt_baro:     Option<i32>,          // feet MSL
    lat:          Option<f64>,
    lon:          Option<f64>,
    gs:           Option<f64>,          // ground speed, knots
    track:        Option<f64>,          // degrees, 0 = north, clockwise
    baro_rate:    Option<i32>,          // feet per minute, negative = descending
    cpr_even:     Option<(u32, u32)>,   // (lat17, lon17) from the most recent even frame
    cpr_odd:      Option<(u32, u32)>,   // (lat17, lon17) from the most recent odd frame
    last_even_ts: Option<Instant>,
    last_odd_ts:  Option<Instant>,
    last_seen:    Instant,
    msg_count:    u32,
}
```

On each incoming DF17 message the tracker:
1. Looks up (or creates) the `AircraftState` for the message's ICAO address.
2. Updates `last_seen` and increments `msg_count`.
3. Dispatches on TC:
   - TC 1–4 → update `callsign`
   - TC 9–18 → update `alt_baro`, store the CPR frame, attempt a position decode
   - TC 19 → update `gs`, `track`, `baro_rate`

CPR position is attempted whenever both `last_even_ts` and `last_odd_ts` are set and the older one is less than 10 seconds old. If successful, `lat` and `lon` are updated. If the decode fails (inconsistent frames, zone boundary straddle), the frames are kept in case the next message completes a valid pair.

#### Purging

Every write cycle the tracker removes aircraft whose `last_seen` is more than 60 seconds ago. Without purging, aircraft that fly out of range stay in the JSON indefinitely, creating a ghost-filled map. 60 seconds is long enough to survive a brief signal dropout while short enough to clear aircraft that have genuinely left the area.

---

### Writing aircraft.json

tar1090 polls `aircraft.json` from a configured directory every ~8 seconds. The file format is a JSON object:

```json
{
  "now":      1712345678.912,
  "messages": 1547,
  "aircraft": [
    {
      "hex":      "4ca7ed",
      "flight":   "AMC421  ",
      "lat":      35.8792,
      "lon":      14.4731,
      "alt_baro": 1050,
      "gs":       148.3,
      "track":    272.1,
      "baro_rate": -768,
      "seen":     0.4,
      "seen_pos": 0.4,
      "messages": 38
    }
  ]
}
```

Field semantics:
- `now`: Unix timestamp (seconds, float) when the file was written.
- `messages`: cumulative total messages decoded across all aircraft since startup.
- `hex`: ICAO address, 6 lowercase hex digits, no `0x` prefix.
- `flight`: callsign, 8 characters including trailing spaces. tar1090 strips them in the UI.
- `lat`/`lon`: decimal degrees, 6 decimal places (≈ 0.1 m resolution — more than adequate for ADS-B accuracy).
- `alt_baro`: barometric altitude, feet MSL. Integer.
- `gs`: ground speed, knots.
- `track`: heading, degrees clockwise from north.
- `baro_rate`: vertical rate, feet per minute. Negative means descending. tar1090 uses this to draw the climb/descent indicator on the aircraft label.
- `seen`: seconds since any message received. tar1090 uses this to fade old icons.
- `seen_pos`: seconds since the last position update specifically. A plane may have recent speed data but a stale position if it stopped transmitting TC 9–18.
- `messages`: per-aircraft message count. tar1090 uses this as a proxy for signal quality.

Fields with `None` values (no data yet for this aircraft) are simply omitted from the JSON. tar1090 handles missing fields gracefully — an aircraft with a position but no callsign still appears on the map.

The file is written atomically: first to `aircraft.json.tmp`, then renamed to `aircraft.json`. This is the standard POSIX atomic write pattern — a rename is atomic on Linux, so tar1090 can never read a partially-written file. Without this, a slow write could give tar1090 truncated JSON, crashing its parser and blanking the map.

The write happens every 1 second from a 1 Hz timer inside the state tracker thread. For file input (not live SDR), a final write is issued after the decoder finishes processing the file, ensuring the map updates even if the file is shorter than 1 second of data.

---

## Bug Record: NL Table Off-By-One (CPR Longitude Error)

**Symptom:** Decoded aircraft positions were consistently ~2 nm west of their true position. Latitude was accurate; only longitude was wrong. The offset was stable (not random), which ruled out bit-extraction errors and pointed to a systematic zone-count error.

**Root cause:** The NL lookup table was constructed with incorrect values — each entry stored the NL value for the *current* latitude band rather than the *next* band. This meant `nl(lat)` returned `NL - 1` for every latitude, shifting the longitude zone count by one.

For example, at lat 37°N (near Malta):
- Correct `nl()`: **47** zones
- Buggy `nl()`: **46** zones

The longitude decode uses `ni = nl_val - (0 or 1)` as the number of longitude zones, and `dlon = 360 / ni`. With `ni = 46` instead of 47, each zone is ~0.30° wide instead of ~0.29°, accumulating into a ~0.3° (~18 nm at that latitude) maximum error, reduced by the fractional CPR offset to ~2 nm in practice.

**Why latitude was unaffected:** CPR latitude uses fixed zone counts (60 even, 59 odd) hardcoded in the algorithm — they don't go through `nl()`. Only longitude uses `nl()`.

**The fix:** Replace the lookup table with an explicit `if-else` chain matching the ICAO Annex 10 Appendix B thresholds exactly, as used by dump1090 and readsb. The table approach is error-prone because the threshold for zone N is the latitude *above which* you enter zone N-1, not zone N — easy to misread.

```rust
// Correct: threshold is the lower bound of the *next* (smaller) NL value
fn nl(lat: f64) -> u32 {
    let lat = lat.abs();
    if lat < 10.47047130 { return 59; }
    if lat < 14.82817437 { return 58; }
    // ... etc
    1
}
```

**Lesson:** When implementing CPR, validate `nl()` against the reference table in dump1090's `cpr.c` before doing anything else. A unit test asserting `nl(51.5) == 36` (London) and `nl(0.0) == 59` (equator) would have caught this immediately.
