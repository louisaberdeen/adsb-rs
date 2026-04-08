import asyncio
from rtlsdr import RtlSdr
import numpy as np
from dataclasses import dataclass
from typing import Tuple


async def rtl_streaming(out_queue: asyncio.Queue):
    sdr = RtlSdr()
    sdr.sample_rate = 2.4e6
    sdr.center_freq = 1090e6
    sdr.gain = 'auto'

    try:
        async for samples in sdr.stream(num_samples_or_bytes=64*1024):
            magnitude = np.abs(samples)
            magnitude = (magnitude * 65535 / np.sqrt(2)).astype(np.uint16)
            await out_queue.put(magnitude)
    except asyncio.CancelledError:
        print("Streaming cancelled")
    finally:
        await sdr.stop()
        sdr.close()
        print("SDR closed")


@dataclass
class SampleChunk:
    samples: np.ndarray
    offset: int  # Global sample offset in file


async def file_streaming(file_name: str, out_queue: asyncio.Queue, realtime: bool = False):
    sample_rate = 2.4e6
    chunk_bytes = 64 * 1024
    global_offset = 0

    try:
        with open(file_name, 'rb') as f:
            while True:
                raw = f.read(chunk_bytes)
                if not raw:
                    break

                data = np.frombuffer(raw, dtype=np.uint8)

                if len(data) % 2 != 0:
                    data = data[:-1]

                # Convert 8-bit unsigned IQ to magnitude matching readsb's init_uc8_lookup()
                i_samples = (data[0::2].astype(np.float32) - 127.5) / 127.5
                q_samples = (data[1::2].astype(np.float32) - 127.5) / 127.5
                magsq = np.minimum(i_samples * i_samples + q_samples * q_samples, 1.0)
                magnitude = (np.sqrt(magsq) * 65535.0 + 0.5).astype(np.uint16)

                await out_queue.put(SampleChunk(magnitude, global_offset))
                global_offset += len(magnitude)

                if realtime:
                    num_samples = len(magnitude)
                    delay = num_samples / sample_rate
                    await asyncio.sleep(delay)

        await out_queue.put(None)
        print("File streaming complete")
    except asyncio.CancelledError:
        print("File streaming cancelled")


def compute_preamble_magnitude(samples: np.ndarray, i: int, phase: int):
    pa = samples[i:i+20]

    if len(pa) < 20:
        return 0

    diff_2_3 = int(pa[2]) - int(pa[3])
    sum_1_4 = int(pa[1]) + int(pa[4])
    diff_10_11 = int(pa[10]) - int(pa[11])
    common3456 = sum_1_4 - diff_2_3 + int(pa[9]) + int(pa[12])

    if phase in [0, 1]:
        pa_mag = common3456 - diff_10_11
    elif phase in [2, 3]:
        pa_mag = common3456 + diff_10_11
    else:
        pa_mag = sum_1_4 + 2 * diff_2_3 + diff_10_11 + int(pa[12])

    return pa_mag


def compute_noise_and_threshold(samples: np.ndarray, i: int, threshold_factor: int = 58):
    # 5 samples that should be quiet during valid preamble
    noise = (int(samples[i+5]) + int(samples[i+8]) +
             int(samples[i+16]) + int(samples[i+17]) + int(samples[i+18]))

    # Threshold = noise x factor / 32  (readsb default factor = 58)
    ref_level = (noise * threshold_factor) >> 5

    return noise, ref_level


@dataclass
class ADSBPayload:
    samples: np.ndarray
    sample_offset: int  # Global position in file


async def preamble_detection(in_queue: asyncio.Queue, out_queue: asyncio.Queue):
    overlap = None  # tail of previous chunk (last 300 samples)
    try:
        while True:
            chunk = await in_queue.get()

            if chunk is None:
                await out_queue.put(None)
                print("Preamble detection complete")
                break

            # Carry over tail from previous chunk so preambles near chunk
            # boundaries aren't missed (same approach as reference basic_stream.py)
            if overlap is not None:
                samples = np.concatenate([overlap, chunk.samples])
                base_offset = chunk.offset - len(overlap)
            else:
                samples = chunk.samples
                base_offset = chunk.offset

            overlap = samples[-300:] if len(samples) > 300 else None

            for i in range(len(samples) - 300):

                # Quick pre-check to skip expensive correlation (matches readsb)
                if not (samples[i+1] > samples[i+7] and
                        samples[i+12] > samples[i+14] and
                        samples[i+12] > samples[i+15]):
                    continue

                noise, ref_level = compute_noise_and_threshold(samples, i, 58)

                # Fire if any phase group passes the threshold (>= matches readsb)
                any_pass = False
                for phase in range(5):
                    pa_mag = compute_preamble_magnitude(samples, i, phase)
                    if pa_mag >= ref_level:
                        any_pass = True
                        break

                if any_pass:
                    global_pos = base_offset + i
                    await out_queue.put(ADSBPayload(samples[i:i+300], global_pos))

    except asyncio.CancelledError:
        print("Preamble detection cancelled")


def decode_bit_correlation(samples: np.ndarray, pos: int, phase: int) -> int:
    """Phase-aware correlation for 2.4 MHz — matches readsb's slice_phase0..4."""
    m = samples[pos:pos+4]
    if len(m) < 3:
        return 0

    m0, m1, m2 = int(m[0]), int(m[1]), int(m[2])

    if phase == 0:
        corr = 18*m0 - 15*m1 - 3*m2
    elif phase == 1:
        corr = 14*m0 - 5*m1 - 9*m2
    elif phase == 2:
        corr = 16*m0 + 5*m1 - 20*m2
    elif phase == 3:
        corr = 7*m0 + 11*m1 - 18*m2
    else:
        m3 = int(m[3]) if len(m) > 3 else 0
        corr = 4*m0 + 15*m1 - 20*m2 + m3

    return 1 if corr > 0 else 0


def decode_bits(samples: np.ndarray, start_pos: int, detect_phase: int, num_bits: int):
    # Each bit is exactly 12/5 samples wide at 2.4 MHz.
    # Bit n starting at sub-sample phase p has:
    #   sample offset = (12*n + p) // 5   (exact integer arithmetic, no float error)
    #   correlation phase = (p + 2*n) % 5
    # This avoids accumulated floating-point error from repeatedly adding 2.4.
    p = (detect_phase + 4) % 5  # starting sub-sample phase (0..4)
    bits = np.zeros(num_bits, dtype=np.uint8)
    for n in range(num_bits):
        offset = (12 * n + p) // 5
        bit_phase = (p + 2 * n) % 5
        bits[n] = decode_bit_correlation(samples=samples, pos=start_pos + offset, phase=bit_phase)
    return bits


VALID_DF_SHORT = {0, 4, 5, 11}
VALID_DF_LONG  = {16, 17, 18, 20, 21, 24}


def try_decode_phase(samples: np.ndarray, phase: int) -> Tuple[int, bytearray]:
    """Try decoding a message at a given phase. Returns (df, msg) or None."""
    # phase 0 starts 1 sample earlier — matches readsb's score_phase() pointer offset
    preamble_samples = 19 if phase == 0 else 20

    # Decode first 5 bits to get Downlink Format
    first_bits = decode_bits(samples=samples, start_pos=preamble_samples,
                             detect_phase=phase, num_bits=5)
    df = 0
    for b in first_bits:
        df = (df << 1) | b

    if df in VALID_DF_LONG:
        payload_length = 112
    elif df in VALID_DF_SHORT:
        payload_length = 56
    else:
        return None

    payload_bits = decode_bits(samples=samples, start_pos=preamble_samples,
                               detect_phase=phase, num_bits=payload_length)

    num_bytes = payload_length // 8
    msg = bytearray(num_bytes)
    for i in range(num_bytes):
        byte_val = 0
        for bit in payload_bits[i*8:(i+1)*8]:
            byte_val = (byte_val << 1) | bit
        msg[i] = byte_val

    return df, msg


async def decoder(in_q: asyncio.Queue, output_file: str = None):
    icao_filter = set()
    file_handle = open(output_file, 'w') if output_file else None

    # Suppress duplicate detections of the same physical message.
    # preamble_detection fires at many adjacent positions for one transmission;
    # after accepting a message, skip forward by msglen*8/4 samples — matching
    # readsb's post-decode pa += msglen*8/4.  Use last_skip_end (not a fixed
    # window) so long messages (skip=224) and short messages (skip=112) are
    # handled correctly, just like readsb.
    last_skip_end = -1000  # sample position where the skip window ends

    def output(line):
        print(line)
        if file_handle:
            file_handle.write(line + '\n')

    try:
        while True:
            payload = await in_q.get()

            if payload is None:
                print("Decoder complete")
                break

            samples = payload.samples
            sample_pos = payload.sample_offset

            # Suppress duplicates: skip if within the skip window of last accepted
            if sample_pos < last_skip_end:
                continue

            best_hex = None
            best_score = -1
            best_payload_length = 112

            for phase in range(5):
                result = try_decode_phase(samples=samples, phase=phase)
                if result is None:
                    continue

                df, msg = result
                crc = modes_checksum(msg)

                if df in (17, 18):
                    if crc != 0:
                        continue
                    icao = (msg[1] << 16) | (msg[2] << 8) | msg[3]
                    icao_filter.add(icao)
                    score = 1800 if icao in icao_filter else 1400

                elif df == 11:
                    if crc & 0xFFFF80:          # bottom 7 bits = IID, top 17 must be 0
                        continue
                    icao = (msg[1] << 16) | (msg[2] << 8) | msg[3]
                    icao_filter.add(icao)
                    score = 1000

                elif df in (0, 4, 5, 16, 20, 21, 24):
                    # CRC residual IS the ICAO — accept only if we've seen that aircraft
                    if crc not in icao_filter:
                        continue
                    score = 800

                else:
                    continue

                if score > best_score:
                    best_score = score
                    best_hex = msg.hex()
                    best_payload_length = len(msg) * 8

            if best_hex is not None:
                output(f"@{sample_pos:08d} *{best_hex};")
                last_skip_end = sample_pos + (best_payload_length * 8 // 4)

    except asyncio.CancelledError:
        print("Payload decoder cancelled")
    finally:
        if file_handle:
            file_handle.close()


def modes_checksum(msg: bytes) -> int:
    POLY = 0xFFF409
    crc = 0
    for byte in msg[:-3]:
        crc ^= (byte << 16)
        for _ in range(8):
            if crc & 0x800000:
                crc = (crc << 1) ^ POLY
            else:
                crc <<= 1
            crc &= 0xFFFFFF
    crc ^= (msg[-3] << 16) | (msg[-2] << 8) | msg[-1]
    return crc


async def main():
    raw_q = asyncio.Queue(maxsize=100)
    payload_q = asyncio.Queue(maxsize=1024)
    tasks = [
        asyncio.create_task(file_streaming("data/modes1_2.4mhz.bin", raw_q)),
        asyncio.create_task(preamble_detection(raw_q, payload_q)),
        asyncio.create_task(decoder(payload_q, output_file="decoder_output.txt")),
    ]

    await asyncio.gather(*tasks)
    print("Shutdown complete")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nStopped")
