#!/usr/bin/env python3
"""
Resample 8-bit unsigned IQ samples between different sample rates.

Usage: python resample_iq.py input.bin output.bin --from-rate 2000000 --to-rate 2400000
"""

import argparse
import numpy as np
from math import gcd
from scipy.signal import resample_poly


def main():
    parser = argparse.ArgumentParser(
        description="Resample 8-bit unsigned IQ samples"
    )
    parser.add_argument("input", help="Input IQ file (8-bit unsigned)")
    parser.add_argument("output", help="Output IQ file (8-bit unsigned)")
    parser.add_argument(
        "--from-rate", type=int, required=True, help="Input sample rate in Hz"
    )
    parser.add_argument(
        "--to-rate", type=int, required=True, help="Output sample rate in Hz"
    )
    args = parser.parse_args()

    # Read input file as 8-bit unsigned
    raw = np.fromfile(args.input, dtype=np.uint8)

    # Convert interleaved I/Q to complex float64 (centered around 0)
    i_samples = raw[0::2].astype(np.float64) - 127.5
    q_samples = raw[1::2].astype(np.float64) - 127.5
    complex_samples = i_samples + 1j * q_samples

    # Calculate resampling ratio (simplify to integers)
    divisor = gcd(args.to_rate, args.from_rate)
    up = args.to_rate // divisor
    down = args.from_rate // divisor

    print(f"Input samples: {len(complex_samples)}")
    print(f"Resampling ratio: {up}/{down} ({args.from_rate} Hz -> {args.to_rate} Hz)")

    # Resample using polyphase filter
    resampled = resample_poly(complex_samples, up, down)

    print(f"Output samples: {len(resampled)}")

    # Convert back to 8-bit unsigned interleaved I/Q
    i_out = np.real(resampled) + 127.5
    q_out = np.imag(resampled) + 127.5

    # Clip to valid range and convert to uint8
    i_out = np.clip(i_out, 0, 255).astype(np.uint8)
    q_out = np.clip(q_out, 0, 255).astype(np.uint8)

    # Interleave I and Q samples
    output = np.empty(len(i_out) * 2, dtype=np.uint8)
    output[0::2] = i_out
    output[1::2] = q_out

    # Write output file
    output.tofile(args.output)
    print(f"Written to: {args.output}")


if __name__ == "__main__":
    main()
