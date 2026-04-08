import asyncio
from rtlsdr import RtlSdr
import numpy as np
from dataclasses import dataclass
from typing import Tuple


# We start by accessing either a steam of real IQ values from our RTL-SDR
# or from a binary file for debugging
# worth noting that one sample = one byte so chunk size and chunk_bytes are interchangable (I believe) 

@dataclass
class SampleChunk:
    samples: np.ndarray
    count: int  # for debuggin

@dataclass
class EndOfStream:
    pass

@dataclass
class ADSBConfig:
    sample_rate: float = 2.4e6
    center_freq: float = 1090e6
    gain: str = 'auto'
    chunk_size: int = 64*1024
    debug:bool = False
    realtime: bool  = True
    threshold_factor: int = 50
    preamble_skip: int = 5
    use_nms: bool = True

async def rtl_streaming(config: ADSBConfig, out_queue: asyncio.Queue):
    sdr = RtlSdr()
    sdr.sample_rate = config.sample_rate
    sdr.center_freq = config.center_freq
    sdr.gain = config.gain

    chunk_size = config.chunk_size

    sdr.read_samples(2048) # recomended to clear out transients

    count = 0

    try:
        async for iq_samples in sdr.stream(num_samples_or_bytes=chunk_size):
            if len(iq_samples) % 2 != 0:
                iq_samples = iq_samples[:-1]
            # pysdr returns our samples as floats between -1 and 1 helpfully 
            i_samples = (iq_samples[0::2])
            q_samples = (iq_samples[1::2])
            magsq = np.minimum(i_samples * i_samples + q_samples * q_samples, 1.0)      # clamp to 1.0
            magnitude = (np.sqrt(magsq) * 65535.0 + 0.5).astype(np.uint16)

            await out_queue.put(SampleChunk(magnitude, count))
            count += 1   
    except asyncio.CancelledError:
        print("Streaming cancelled")
    finally:
        await sdr.stop()
        sdr.close() # good practice to stop the RTL-SDR freaking out
        print("SDR closed")


async def file_streaming(config: ADSBConfig, file_name: str, out_queue: asyncio.Queue, realtime: bool = False):
    sample_rate = config.sample_rate
    chunk_bytes = config.chunk_size
    count = 0

    try:
        with open(file_name, 'rb') as f:
            while True:
                raw = f.read(chunk_bytes)
                if not raw:
                    break

                # convert bytes to numpy array of uint8
                data = np.frombuffer(raw, dtype=np.uint8)

                # ensure even number of samples (I/Q pairs)
                if len(data) % 2 != 0:
                    data = data[:-1]

                # convert 8-bit unsigned IQ to complex float (centered around 0)
                i_samples = (data[0::2].astype(np.float32) - 127.5) / 127.5
                q_samples = (data[1::2].astype(np.float32) - 127.5) / 127.5
                magsq = np.minimum(i_samples * i_samples + q_samples * q_samples, 1.0)      # clamp to 1.0
                magnitude = (np.sqrt(magsq) * 65535.0 + 0.5).astype(np.uint16)

                await out_queue.put(SampleChunk(magnitude, count))
                count += 1

                # simulate real-time streaming delay
                if realtime:
                    num_samples = len(magnitude)
                    delay = num_samples / sample_rate
                    await asyncio.sleep(delay)

        # Signal end of stream
        await out_queue.put(EndOfStream())
        print("File streaming complete")
    except asyncio.CancelledError:
        print("File streaming cancelled")

# Now we have our stream of data we need to detect the preamble 
# ADSB uses PPM modulation to encode the message data. symobls are 500ns 
# with two symbols per bit, high-low for 1 and low-high for 0 
# Symbol width: 500 ns
# Sample period: 1/2.4MHz = 416.67 ns
# Ratio: 500/416.67 = 1.2 samples/symbol = 6 samples per 5 symbols

@dataclass
class ADSBCandidate:
    samples: np.ndarray
    best_phase: int
    count: int  # for debuggin


async def preamble_detection(config:ADSBConfig, in_queue: asyncio.Queue, out_queue: asyncio.Queue):
    # We need to keep the trailing ~300 smaples between chunks
    # since messages can and do overlap
    samples_tail = None 
    samples_tail_len = 300
    threshold_factor = config.threshold_factor
    count = 0
    try:
        while True:
            item: SampleChunk | EndOfStream = await in_queue.get()
            match item:
                case EndOfStream():
                    await out_queue.put(EndOfStream())
                    print("Preamble detection complete")
                    break
                case SampleChunk() as sample_chunk:
                    samples = sample_chunk.samples
                    chunk_count = sample_chunk.count

                    if samples_tail is not None:
                        samples = np.concatenate([samples_tail, samples])

                    samples_tail = samples[-samples_tail_len:]

                    # quick check to skip expensive correlation check
                    i = 0
                    while i < (len(samples)-samples_tail_len):
                        if not (samples[i+1] > samples[i+7] and
                        samples[i+12] > samples[i+14] and
                        samples[i+12] > samples[i+15]):
                            i += 1
                            continue
                        else:
                            # now we need to calculate the local noise level and choose an
                            # appropriate noise threshold to detect pulses. Within a
                            # valid preamble there should be 5 samples with are low no 
                            # matter the phase
                            noise, ref_level = compute_noise_and_threshold(samples, i, threshold_factor)

                            best_phase = -1
                            best_mag = 0

                            for phase in range(5):
                                pa_mag = compute_preamble_magnitude(samples, i, phase)
                                if pa_mag > best_mag and pa_mag > ref_level:
                                    best_mag = pa_mag
                                    best_phase = phase

                            if best_phase >= 0:
                                # Need more than 288 because depending on the phase the overlap shifts
                                # We keep track of the best phase as it's cheap to compute and we will use it
                                # to save iterations in the decoder
                                await out_queue.put(ADSBCandidate(samples[i:i+300], best_phase, count)) 
                                i += config.preamble_skip
                                # skip past this preamble
                            i+=1
            
    except asyncio.CancelledError:
        print(f"File streaming cancelled")
        if config.debug:
            print(f"Chunks processed: {chunk_count}")


async def preamble_detection_nms(config: ADSBConfig, in_queue: asyncio.Queue, out_queue: asyncio.Queue):
    # Same as preamble_detection but uses Non-Maximum Suppression instead of a blind skip.
    # Rather than emitting every candidate and jumping N samples ahead, we track the
    # strongest candidate within a sliding window of size nms_window.  Only when the
    # next detection lands outside that window do we emit the window's winner and open
    # a new one.  This prevents a weak false-positive from consuming the skip budget
    # and hiding a stronger real preamble just ahead.
    samples_tail = None
    samples_tail_len = 300
    threshold_factor = config.threshold_factor
    nms_window = config.preamble_skip
    count = 0
    try:
        while True:
            item: SampleChunk | EndOfStream = await in_queue.get()
            match item:
                case EndOfStream():
                    await out_queue.put(EndOfStream())
                    print("Preamble detection complete")
                    break
                case SampleChunk() as sample_chunk:
                    samples = sample_chunk.samples
                    chunk_count = sample_chunk.count

                    if samples_tail is not None:
                        samples = np.concatenate([samples_tail, samples])

                    samples_tail = samples[-samples_tail_len:]

                    win_i = None    # index of the current window's best candidate
                    win_mag = 0
                    win_phase = -1

                    i = 0
                    while i < (len(samples) - samples_tail_len):
                        if not (samples[i+1] > samples[i+7] and
                                samples[i+12] > samples[i+14] and
                                samples[i+12] > samples[i+15]):
                            i += 1
                            continue

                        noise, ref_level = compute_noise_and_threshold(samples, i, threshold_factor)

                        best_phase = -1
                        best_mag = 0

                        for phase in range(5):
                            pa_mag = compute_preamble_magnitude(samples, i, phase)
                            if pa_mag > best_mag and pa_mag > ref_level:
                                best_mag = pa_mag
                                best_phase = phase

                        if best_phase >= 0:
                            if win_i is None:
                                # Open a new window at this candidate
                                win_i, win_mag, win_phase = i, best_mag, best_phase
                            elif i - win_i <= nms_window:
                                # Inside the window — keep whichever is stronger
                                if best_mag > win_mag:
                                    win_i, win_mag, win_phase = i, best_mag, best_phase
                            else:
                                # Outside the window — emit the winner, open a new window
                                await out_queue.put(ADSBCandidate(samples[win_i:win_i+300], win_phase, count))
                                count += 1
                                win_i, win_mag, win_phase = i, best_mag, best_phase
                        i += 1

                    # Flush the last window winner at end of chunk.
                    # Candidates near the boundary may be re-detected via samples_tail in the
                    # next chunk, but the decoder deduplicates through the ICAO filter.
                    if win_i is not None:
                        await out_queue.put(ADSBCandidate(samples[win_i:win_i+300], win_phase, count))
                        count += 1

    except asyncio.CancelledError:
        print(f"File streaming cancelled")
        if config.debug:
            print(f"Chunks processed: {chunk_count}")


def compute_noise_and_threshold(samples: np.ndarray, i: int, threshold_factor: int = 52):
    # 5 samples that should be quiet during valid preamble
    noise = (int(samples[i+5]) + int(samples[i+8]) +
             int(samples[i+16]) + int(samples[i+17]) + int(samples[i+18]))

    # Threshold = noise × factor / 32
    ref_level = (noise * threshold_factor) >> 5

    return noise, ref_level

def compute_preamble_magnitude(samples: np.ndarray, i: int, phase: int):
    # The preamble magnitude comes from the correlation between the 
    # preamble matched filter and the recieved samples  
    # From readsb comment:
    #   sample#:  0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19
    #   phase 0:  2  4  0  5  1  0  0  0  0  5  1  3  3  0  0  0  0  0  0  0
    #   phase 1:  1  5  0  4  2  0  0  0  0  4  2  2  4  0  0  0  0  0  0  0
    #   phase 2:  0  5  1  3  3  0  0  0  0  3  3  1  5  0  0  0  0  0  0  0
    #   phase 3:  0  4  2  2  4  0  0  0  0  2  4  0  5  1  0  0  0  0  0  0
    #   phase 4:  0  3  3  1  5  0  0  0  0  1  5  0  4  2  0  0  0  0  0  0     
    #   so we could turn this into a template where for any given phase
    #   template = PREAMBLE_TEMPLATES[phase]
    #   return sum(template[n] * int(samples[i + n]) for n in range(20))
    # The simplifction is to just use +1 for on pulse and -1 for off pulse
    # then use algebra to collect like terms to this very neat function
                                                                                                                                                                                                                                                                         
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

# Now we need to decode our candiate chunks of IQ samples, we use our best phase as a 
# starting point. We will then for each candidate phase attempt to decode, if it passes
# the crc error check then we accept it.

def spiral_from(start: int, n: int) -> list[int]:
    result = [start]
    for delta in range(1, n):
        if start - delta >= 0:
            result.append(start - delta)
        if start + delta < n:
            result.append(start + delta)
    return result

def decode_bit_correlation(samples: np.ndarray, sample_offset: int, subsample_phase: int) -> int:
    # Extract a small window of samples around this bit position.
    # We need up to 4 samples because phase 4 uses a 4-sample correlation kernel.
    window = samples[sample_offset:sample_offset+4]
    if len(window) < 3:
        return 0

    # Each ADS-B bit is a Manchester-encoded pulse: high then low (1) or low then high (0).
    # The correlation coefficients below are pre-computed weights that match the expected
    # pulse shape at each sub-sample phase offset (0–4 at 2.4 MHz / 1 Mcps = 2.4 samples/chip).
    # Positive correlation → bit is 1, negative → bit is 0.
    s0, s1, s2 = int(window[0]), int(window[1]), int(window[2])

    if subsample_phase == 0:
        correlation = 18*s0 - 15*s1 - 3*s2
    elif subsample_phase == 1:
        correlation = 14*s0 - 5*s1 - 9*s2
    elif subsample_phase == 2:
        correlation = 16*s0 + 5*s1 - 20*s2
    elif subsample_phase == 3:
        correlation = 7*s0 + 11*s1 - 18*s2
    else:
        # Phase 4 kernel straddles an extra sample boundary
        s3 = int(window[3]) if len(window) > 3 else 0
        correlation = 4*s0 + 15*s1 - 20*s2 + s3

    return 1 if correlation > 0 else 0

def decode_bits(samples: np.ndarray, start_sample: int, phase: int, num_bits: int):
    # At 2.4 MHz each bit is exactly 2.4 samples wide (1 Mcps chip rate).
    # Rather than accumulating 2.4 as a float, we use integer arithmetic:
    #   sample_offset of bit n = (12*n + data_phase) // 5
    #   subsample_phase of bit n = (2*n + data_phase) % 5
    # where 12/5 = 2.4 and 2*n tracks the phase accumulation across bits.
    #
    # The data_phase is shifted by -1 (mod 5) from the preamble_phase to
    # account for the offset between where the preamble correlator locks
    # and where the first data bit actually begins.
    data_phase = (phase + 4) % 5  # +4 ≡ -1 mod 5

    bits = np.zeros(num_bits, dtype=np.uint8)
    for bit_idx in range(num_bits):
        sample_offset = (12 * bit_idx + data_phase) // 5
        subsample_phase = (2 * bit_idx + data_phase) % 5
        bits[bit_idx] = decode_bit_correlation(samples, start_sample + sample_offset, subsample_phase)
    return bits

VALID_DF_SHORT = {0, 4, 5, 11}
VALID_DF_LONG  = {16, 17, 18, 19, 20, 21, 24}

def try_decode_phase(samples: np.ndarray, trial_phase: int):
    # phase 0 starts 1 sample earlier — matches readsb's score_phase() pointer offset
    preamble_samples = 19 if trial_phase == 0 else 20

    first_bits = decode_bits(samples=samples, start_sample=preamble_samples, phase=trial_phase, num_bits=5)

    df = 0
    # extract our DF code from the first 5 bits using bit shifting
    for b in first_bits:
        df = (df << 1) | b


    if df in VALID_DF_LONG:
        payload_length = 112
    elif df in VALID_DF_SHORT:
        payload_length = 56
    else:
        return None
    
    payload_bits = decode_bits(samples=samples, start_sample=preamble_samples,
                               phase=trial_phase, num_bits=payload_length)

    num_bytes = payload_length // 8
    msg = bytearray(num_bytes)
    for i in range(num_bytes):
        byte_val = 0
        for bit in payload_bits[i*8:(i+1)*8]:
            byte_val = (byte_val << 1) | bit
        msg[i] = byte_val

    return df, msg


def modes_checksum(msg: bytes) -> int:
    # Mode S uses a 24-bit CRC with generator polynomial x^24 + x^23 + x^10 + x^3 + 1,
    # represented as 0xFFF409 (24 lower bits, MSB-first, implicit x^24 term dropped).
    POLY = 0xFFF409
    crc = 0
    # Process all bytes except the last 3, which hold the transmitted CRC.
    for byte in msg[:-3]:
        crc ^= (byte << 16)  # XOR byte into the top 8 bits of the 24-bit shift register
        for _ in range(8):
            if crc & 0x800000:  # MSB set: shift out a 1, XOR in the polynomial
                crc = (crc << 1) ^ POLY
            else:
                crc <<= 1
            crc &= 0xFFFFFF  # Keep only 24 bits
    # XOR the computed CRC against the 3 transmitted CRC bytes.
    # Result is 0 if the message is valid, non-zero if corrupted.
    crc ^= (msg[-3] << 16) | (msg[-2] << 8) | msg[-1]
    return crc

async def decoding(config: ADSBConfig, in_queue: asyncio.Queue):
    VALID_DFS = {0, 4, 5, 11, 16, 17, 18, 19, 20, 21, 24}
    icao_filter = set()
    # We now ingest or candiate streams of samples for 
    try:
        while True:
            item: ADSBCandidate | EndOfStream = await in_queue.get()
            match item:
                case EndOfStream():
                    print("Decoding complete")
                    break
                case ADSBCandidate() as candidate:
                    samples = candidate.samples
                    preamble_phase = candidate.best_phase
                    candidate_count = candidate.count

                    best_hex = None
                    best_score = -1
                    best_payload_length = 112
                    decode_phase = None  # phase that produced the accepted decode

                    for phase in spiral_from(preamble_phase, 5):
                        result = try_decode_phase(samples, phase)

                        if result is None:
                            continue

                        df, msg = result
                        crc = modes_checksum(msg)

                        if df in (17, 18):
                            if crc != 0:
                                continue
                            icao = (msg[1] << 16) | (msg[2] << 8) | msg[3]
                            score = 1800 if icao in icao_filter else 1400
                            icao_filter.add(icao)


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
                            decode_phase = phase

                        # CRC passed and score recorded — no need to try remaining phases
                        break

                    if best_hex is not None:
                        if config.debug:
                            phase_match = "exact" if decode_phase == preamble_phase else f"delta={decode_phase - preamble_phase:+d}"
                            print(f"{best_hex}  [preamble_phase={preamble_phase} decode_phase={decode_phase} {phase_match}]")
                        else:
                            print(f"{best_hex}")

                        
    except asyncio.CancelledError:
        print(f"File streaming cancelled")
        if config.debug:
            print(f"Chunks processed: {candidate_count}")

async def main():
    config = ADSBConfig()
    config.debug = True
    raw_q = asyncio.Queue(maxsize=100)
    candidate_q = asyncio.Queue(maxsize=100)
    if config.debug == True:
        tasks = [
            asyncio.create_task(file_streaming(config,"data/modes1_2.4mhz.bin", raw_q)),
        ]
    else:
        tasks = [
            asyncio.create_task(rtl_streaming(config, raw_q)),
        ]
    detect = preamble_detection_nms if config.use_nms else preamble_detection
    tasks.append(asyncio.create_task(detect(config, raw_q, candidate_q)))
    tasks.append(asyncio.create_task(decoding(config, candidate_q)))
    await asyncio.gather(*tasks)
    print("Shutdown complete")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nStopped")