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
