# ADS-B Decoder

A Python implementation of an ADS-B (Automatic Dependent Surveillance–Broadcast) decoder built from scratch. Decodes raw IQ samples from an RTL-SDR receiver or binary test files into Mode S aircraft messages containing ICAO addresses, positions, altitudes, and velocities.

## What is ADS-B?

Aircraft broadcast their telemetry every ~1 second on 1090 MHz using the Mode S protocol. This project decodes those signals from raw RF samples without relying on librtlsdr's built-in demodulation.

## Project Structure

```
├── adsb_24_simple.py      # Main decoder with Non-Maximum Suppression (NMS)
├── basic_stream.py        # Reference decoder (simpler, no NMS)
├── guide.md               # Full technical documentation
├── compare.sh             # Test harness comparing output against readsb
├── data/
│   ├── modes1_2.4mhz.bin  # Test IQ data (UC8, 2.4 MHz)
│   ├── modes1.bin         # Original test data (2 MHz)
│   ├── readsb_output.txt  # Ground truth from readsb
│   ├── decoder_output.txt # Sample decoder output
│   └── resample_iq.py     # Utility to resample IQ data
└── adsb_rs/               # Rust port (scaffolding, WIP)
```

## Dependencies

```bash
pip install numpy scipy pyrtlsdr
```

- **numpy** — signal processing
- **scipy** — polyphase resampling (for `resample_iq.py`)
- **pyrtlsdr** — RTL-SDR hardware support (optional)

For comparison testing, install `readsb`:
```bash
sudo apt install readsb
```

## Usage

### Decode from test file

```bash
source .venv/bin/activate
python3 adsb_24_simple.py
```

Outputs decoded hex messages to stdout, one per line.

### Decode from live RTL-SDR

In `adsb_24_simple.py`, switch `main()` to use `rtl_streaming` instead of `file_streaming`, then run:

```bash
python3 adsb_24_simple.py
```

### Compare against readsb ground truth

```bash
chmod +x compare.sh
./compare.sh
```

### Resample test data

```bash
python3 data/resample_iq.py data/modes1.bin data/modes1_2.4mhz.bin \
    --from-rate 2000000 --to-rate 2400000
```

## How It Works

The decoder runs a three-stage async pipeline:

1. **Streaming** — reads raw UC8 IQ samples, computes magnitudes
2. **Preamble detection** — finds the 8-microsecond Mode S preamble using phase-aware correlation and NMS to suppress false positives
3. **Decoding** — Manchester-decodes bits, validates CRC-24, and filters by known ICAO addresses

**Why 2.4 MHz?** At 2 MHz (integer ratio), RTL-SDR clock drift causes timing errors. At 2.4 MHz, each bit spans exactly 12/5 samples, producing 5 stable sub-sample phases with zero accumulated drift.

**NMS vs blind skip:** After detecting a preamble candidate, naively skipping forward a fixed number of samples ("blind skip") causes missed messages when a weak false positive consumes the skip budget. Non-Maximum Suppression tracks candidates in a sliding window and emits only the strongest, recovering ~165/168 messages vs readsb's reference output.

## Configuration

Key parameters in `ADSBConfig` (in `adsb_24_simple.py`):

| Parameter | Default | Description |
|---|---|---|
| `sample_rate` | 2.4e6 | SDR sample rate (Hz) |
| `threshold_factor` | 50 | Preamble SNR threshold multiplier |
| `preamble_skip` | 5 | NMS window size |
| `use_nms` | True | Use NMS (True) or blind skip (False) |
| `debug` | False | Print phase mismatch diagnostics |

## Performance

Tested on `modes1_2.4mhz.bin` (~178 ms of traffic):

| Decoder | Unique messages |
|---|---|
| readsb (reference) | 168 |
| adsb_24_simple.py | 165 |
| basic_stream.py | 166 |

## Technical Reference

See [guide.md](guide.md) for a complete walkthrough of the signal processing, preamble structure, Manchester encoding, CRC-24 validation, ICAO filtering, and all design decisions.
