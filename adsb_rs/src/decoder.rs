pub fn spiral_from(start: u8, n: u8) -> Vec<usize> {
    let mut result = vec![start as usize];
    for i in 1..n {
        if start >= i {
            result.push((start - i) as usize);
        }
        if start + i < n {
            result.push((start + i) as usize);
        }
    }
    result
}

pub struct DecodeResult {
    pub df:    u8,
    pub bytes: [u8; 14],
    pub len:   u8,
}

pub fn decode_bit_correlation(samples: &[u16], sample_offset: usize, subsample_phase: u8) -> u8 {
    let window = &samples[sample_offset..];
    if window.len() < 3 { return 0; }

    let s0 = window[0] as i32;
    let s1 = window[1] as i32;
    let s2 = window[2] as i32;

    // Correlation coefficients from dump1090's subsample-phase demodulator.
    // Each row weights three consecutive samples to determine bit polarity
    // at a given fractional sample offset (0..=4 fifths of a symbol period).
    let correlation = match subsample_phase {
        0 => 18*s0 - 15*s1 -  3*s2,
        1 => 14*s0 -  5*s1 -  9*s2,
        2 => 16*s0 +  5*s1 - 20*s2,
        3 =>  7*s0 + 11*s1 - 18*s2,
        _ => {
            let s3 = if window.len() > 3 { window[3] as i32 } else { 0 };
            4*s0 + 15*s1 - 20*s2 + s3
        }
    };

    if correlation > 0 { 1 } else { 0 }
}

pub fn decode_bits(samples: &[u16], start_offset: usize, phase: u8, num_bits: u8) -> Vec<u8> {
    let mut bits       = vec![0_u8; num_bits as usize];
    let data_phase     = (phase + 4) % 5;

    for bit_idx in 0..num_bits as usize {
        let sample_offset   = (12 * bit_idx + data_phase as usize) / 5;
        let subsample_phase = ((2 * bit_idx + data_phase as usize) % 5) as u8;
        bits[bit_idx] = decode_bit_correlation(
            samples,
            start_offset + sample_offset,
            subsample_phase,
        );
    }
    bits
}

pub const VALID_DF_SHORT: [u8; 4] = [0, 4, 5, 11];
pub const VALID_DF_LONG:  [u8; 7] = [16, 17, 18, 19, 20, 21, 24];

pub fn try_decoding(samples: &[u16], trial_phase: u8) -> Option<DecodeResult> {
    let preamble_samples = if trial_phase == 0 { 19 } else { 20 };

    let first_bits = decode_bits(samples, preamble_samples, trial_phase, 5);
    let mut df: u8 = 0;
    for bit in first_bits {
        df = (df << 1) | bit;
    }

    let payload_length = if VALID_DF_LONG.contains(&df) {
        112
    } else if VALID_DF_SHORT.contains(&df) {
        56
    } else {
        return None;
    };

    let payload_bits = decode_bits(samples, preamble_samples, trial_phase, payload_length);
    let num_bytes    = payload_length / 8;
    let mut msg      = [0u8; 14];
    for i in 0..num_bytes as usize {
        for bit in &payload_bits[i * 8..(i + 1) * 8] {
            msg[i] = (msg[i] << 1) | bit;
        }
    }
    Some(DecodeResult { df, bytes: msg, len: num_bytes })
}

pub const POLY: u32 = 0xFFF409;

pub fn modes_checksum(msg: &[u8]) -> u32 {
    let mut crc: u32 = 0;
    let n = msg.len();

    for &byte in &msg[..n - 3] {
        crc ^= (byte as u32) << 16;
        for _ in 0..8 {
            if crc & 0x800000 != 0 {
                crc = (crc << 1) ^ POLY;
            } else {
                crc <<= 1;
            }
            crc &= 0xFFFFFF;
        }
    }
    crc ^= (msg[n-3] as u32) << 16 | (msg[n-2] as u32) << 8 | msg[n-1] as u32;
    crc
}

pub fn decode_callsign(me: &[u8]) -> String {
    let bits = u64::from_be_bytes([0, 0, me[1], me[2], me[3], me[4], me[5], me[6]]);
    let mut cs = String::with_capacity(8);
    for i in (0..8).rev() {
        let idx = ((bits >> (i * 6)) & 0x3F) as u8;
        cs.push(match idx {
            1..=26  => (b'A' + idx - 1) as char,
            48..=57 => (b'0' + idx - 48) as char,
            _       => ' ',
        });
    }
    cs.trim_end().to_string()
}

pub fn decode_altitude(me: &[u8]) -> Option<i32> {
    // Pack ME into a u64 (with a leading zero byte) so we can extract bits by position.
    // ME bit N (0-indexed from MSB of 56-bit field) sits at u64 bit (55 - N).
    // Altitude field = ME bits 8-19  →  u64 bits 47-36  →  (me_u64 >> 36) & 0xFFF
    let me_u64 = u64::from_be_bytes([0, me[0], me[1], me[2], me[3], me[4], me[5], me[6]]);
    let alt_raw = ((me_u64 >> 36) & 0xFFF) as u32;
    let q_bit = (alt_raw >> 4) & 1;
    if q_bit == 1 {
        // Remove Q-bit: upper 7 bits (bits 11-5) then lower 4 bits (bits 3-0)
        let n = ((alt_raw & 0xFE0) >> 1) | (alt_raw & 0x00F);
        if n == 0 { return None; }
        Some(n as i32 * 25 - 1000)
    } else {
        // Q=0: Gillham/Gray code — TODO: implement; rare in modern traffic
        None
    }
}

pub fn decode_velocity(me: &[u8]) -> Option<(f64, f64, i32)> {
    let st = me[0] & 0x07;
    if st != 1 && st != 2 { return None; }
    // Pack 7 ME bytes into u64 for easy bit extraction
    let b = u64::from_be_bytes([0, me[0], me[1], me[2], me[3], me[4], me[5], me[6]]);
    let dir_ew = (b >> 42) & 1;
    let v_ew_r = (b >> 32) & 0x3FF;
    let dir_ns = (b >> 31) & 1;
    let v_ns_r = (b >> 21) & 0x3FF;
    let vr_sign = (b >> 19) & 1;
    let vr_r    = (b >> 10) & 0x1FF;
    if v_ew_r == 0 || v_ns_r == 0 { return None; }
    let v_ew = (v_ew_r as f64 - 1.0) * if dir_ew != 0 { -1.0 } else { 1.0 };
    let v_ns = (v_ns_r as f64 - 1.0) * if dir_ns != 0 { -1.0 } else { 1.0 };
    let gs    = (v_ew * v_ew + v_ns * v_ns).sqrt();
    let track = v_ew.atan2(v_ns).to_degrees().rem_euclid(360.0);
    let vert  = if vr_r == 0 { 0i32 }
                else { let f = (vr_r as i32 - 1) * 64; if vr_sign != 0 { -f } else { f } };
    Some((gs, track, vert))
}
