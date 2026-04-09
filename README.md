# ADS-B Decoder

Rust ADS-B decoder built from scratch. Reads UC8 IQ samples from any SoapySDR-compatible SDR or a binary file and outputs Mode S hex messages. Can write `aircraft.json` directly for tar1090 — no readsb required.

## Build

All commands from the project root (`~/Documents/ADS-B/`).

```bash
# File decoding only
cargo build --release --manifest-path adsb_rs/Cargo.toml

# With live SDR support (requires libsoapysdr-dev)
sudo apt install libsoapysdr-dev
cargo build --release --manifest-path adsb_rs/Cargo.toml --features sdr
```

Binary: `adsb_rs/target/release/adsb_rs`

## Usage

```bash
# Decode a file
adsb_rs data/modes1_2.4mhz.bin

# Live SDR (auto-detect)
adsb_rs --live

# Specific device
adsb_rs --live --device "driver=rtlsdr"

# Write aircraft.json for tar1090 (file or live)
adsb_rs data/modes1_2.4mhz.bin --json /tmp/adsb_json
adsb_rs --live --json /tmp/adsb_json

# Loop file input (useful for testing with tar1090)
adsb_rs data/modes1_2.4mhz.bin --json /tmp/adsb_json --loop

# Serve AVR stream on TCP port 30002 (readsb-compatible)
adsb_rs --live --serve 30002

# List connected SDR devices
adsb_rs --list-devices

# Debug mode (shows preamble/decode phase per message)
adsb_rs --debug data/modes1_2.4mhz.bin
```

### All options

```
Usage: adsb_rs [OPTIONS] [FILE]

Arguments:
  [FILE]  UC8 IQ sample file to decode (2.4 MHz sample rate)

Options:
  -l, --live              Stream live from a connected SDR
  -d, --device <ARGS>     SoapySDR device args e.g. "driver=rtlsdr" [default: auto]
  -g, --gain <DB>         Gain in dB (omit for auto AGC)
  -r, --rate <HZ>         Sample rate [default: 2400000]
  -f, --freq <HZ>         Centre frequency [default: 1090000000]
      --json <DIR>        Write aircraft.json to DIR every second (tar1090-compatible)
      --serve <PORT>      Serve AVR TCP stream on PORT
      --loop              Loop file input continuously
      --debug             Print preamble/decode phase alongside each message
      --no-nms            Use blind skip instead of non-maximum suppression
      --list-devices      List available SoapySDR devices and exit
  -h, --help              Print help
```

## tar1090 Integration

tar1090 reads `aircraft.json` from a directory. `adsb_rs --json` writes that file directly.

```bash
# Install tar1090 (points it at the directory adsb_rs writes to)
sudo bash -c "$(curl -L https://github.com/wiedehopf/tar1090/raw/master/install.sh)" -- /tmp/adsb_json

# Run decoder
adsb_rs --live --json /tmp/adsb_json

# Open http://localhost/tar1090
```

Test the pipeline before connecting hardware:

```bash
python3 test_tar1090.py   # writes a fake aircraft over London every second
```

## Performance

Tested on `modes1_2.4mhz.bin` (~178 ms of real traffic):

| Decoder | Messages |
|---|---|
| readsb (reference) | 168 |
| adsb_rs | 165 |

## Tests

```bash
cargo test --manifest-path adsb_rs/Cargo.toml
```

## Reference

See [guide.md](guide.md) for signal processing, preamble detection, Manchester decoding, CRC-24, CPR position decode, and design decisions.
