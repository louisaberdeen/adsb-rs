use std::collections::HashSet;
use std::fs::File;
use std::io::{self, BufReader, Read};
use std::sync::Arc;
use std::thread;

use crossbeam_channel::bounded;

#[allow(dead_code)]
struct ADSBConfig {
    sample_rate:      f64,         // Hz — used by soapy_streaming
    center_freq:      f64,         // Hz — ADS-B is 1090 MHz
    gain:             Option<f64>, // None = auto AGC, Some(x) = x dB
    sdr_args:         String,      // SoapySDR device filter e.g. "driver=rtlsdr"
    chunk_size:       usize,       // raw bytes per chunk (2 bytes per IQ sample)
    debug:            bool,
    threshold_factor: u16,
    preamble_skip:    usize,
    use_nms:          bool,
    realtime:         bool,
    file_path:        String,
}

impl Default for ADSBConfig {
    fn default() -> Self {
        Self {
            sample_rate:      2_400_000.0,
            center_freq:      1_090_000_000.0,
            gain:             None,
            sdr_args:         String::new(),
            chunk_size:       65536,
            debug:            false,
            threshold_factor: 50,
            preamble_skip:    5,
            use_nms:          true,
            realtime:         true,
            file_path:        String::from(
                "/home/louis/Documents/ADS-B/data/modes1_2.4mhz.bin",
            ),
        }
    }
}

struct SampleChunk {
    data: Vec<u16>,
    len:  usize,
}

fn read_file(
    config: Arc<ADSBConfig>,
    tx: crossbeam_channel::Sender<SampleChunk>,
) -> io::Result<()> {
    let file = File::open(&config.file_path)?;
    let mut reader = BufReader::new(file);

    let mut chunk      = vec![0u8;  config.chunk_size];
    let mut magnitudes = vec![0u16; config.chunk_size / 2];

    loop {
        let n = reader.read(&mut chunk)?;
        if n == 0 { break; }

        for (idx, pair) in chunk[..n].chunks_exact(2).enumerate() {
            let i     = (pair[0] as f32 - 127.5) / 127.5;
            let q     = (pair[1] as f32 - 127.5) / 127.5;
            let magsq = (i * i + q * q).min(1.0);
            magnitudes[idx] = (magsq.sqrt() * 65535.0 + 0.5) as u16;
        }
        if tx
            .send(SampleChunk { data: magnitudes[..n / 2].to_vec(), len: n / 2 })
            .is_err()
        {
            break;
        }
    }
    Ok(())
}

#[cfg(feature = "sdr")]
fn soapy_streaming(config: Arc<ADSBConfig>, tx: crossbeam_channel::Sender<SampleChunk>) {
    use num_complex::Complex;
    use soapysdr::Direction::Rx;

    let dev = soapysdr::Device::new(config.sdr_args.as_str())
        .expect("SoapySDR: no device found");
    dev.set_sample_rate(Rx, 0, config.sample_rate).expect("set_sample_rate");
    dev.set_frequency(Rx, 0, config.center_freq, "").expect("set_frequency");
    match config.gain {
        None    => dev.set_gain_mode(Rx, 0, true).expect("set AGC"),
        Some(g) => dev.set_gain(Rx, 0, g).expect("set_gain"),
    }

    let num_samples = config.chunk_size / 2;
    let mut stream  = dev.rx_stream::<Complex<f32>>(&[0]).expect("rx_stream");
    stream.activate(None).ok();

    // Flush transients (mirrors Python's sdr.read_samples(2048))
    let mut flush = vec![Complex::new(0f32, 0f32); 2048];
    stream.read(&mut [flush.as_mut_slice()], 1_000_000).ok();

    let mut iq_buf     = vec![Complex::new(0f32, 0f32); num_samples];
    let mut magnitudes = vec![0u16; num_samples];

    loop {
        let n = match stream.read(&mut [iq_buf.as_mut_slice()], 1_000_000) {
            Ok(n)  => n,
            Err(_) => break,
        };
        for (idx, s) in iq_buf[..n].iter().enumerate() {
            let magsq = (s.re * s.re + s.im * s.im).min(1.0);
            magnitudes[idx] = (magsq.sqrt() * 65535.0 + 0.5) as u16;
        }
        if tx
            .send(SampleChunk { data: magnitudes[..n].to_vec(), len: n })
            .is_err()
        {
            break;
        }
    }
    stream.deactivate(None).ok();
}

struct ADSBCandidate {
    data:       Vec<u16>,
    best_phase: u8,
}

fn compute_noise_and_threshold(samples: &[u16], idx: usize, threshold_factor: u16) -> (u32, u32) {
    let noise: u32 = samples[idx + 5]  as u32
        + samples[idx + 8]  as u32
        + samples[idx + 16] as u32
        + samples[idx + 17] as u32
        + samples[idx + 18] as u32;
    let ref_level: u32 = (noise * threshold_factor as u32) >> 5;
    (noise, ref_level)
}

fn compute_preamble_magnitude(samples: &[u16], idx: usize, phase: u16) -> i32 {
    if idx + 20 > samples.len() { return 0; }
    let pa         = &samples[idx..idx + 20];
    let diff_2_3   = pa[2] as i32 - pa[3] as i32;
    let sum_1_4    = pa[1] as i32 + pa[4] as i32;
    let diff_10_11 = pa[10] as i32 - pa[11] as i32;
    let common3456 = sum_1_4 - diff_2_3 + pa[9] as i32 + pa[12] as i32;

    if phase == 0 || phase == 1 {
        common3456 - diff_10_11
    } else if phase == 2 || phase == 3 {
        common3456 + diff_10_11
    } else {
        sum_1_4 + 2 * diff_2_3 + diff_10_11 + pa[12] as i32
    }
}

fn preamble_detection(
    config: Arc<ADSBConfig>,
    rx: crossbeam_channel::Receiver<SampleChunk>,
    tx: crossbeam_channel::Sender<ADSBCandidate>,
) {
    let threshold_factor = config.threshold_factor;
    let nms_window       = config.preamble_skip;
    let samples_tail_len = 300usize;
    let mut work_buf     = vec![0u16; samples_tail_len + config.chunk_size / 2];
    let mut tail_valid = 0usize;

    while let Ok(sample_chunk) = rx.recv() {
        let n_total = tail_valid + sample_chunk.len;
        work_buf[tail_valid..n_total].copy_from_slice(&sample_chunk.data[..sample_chunk.len]);

        let tail_start = n_total.saturating_sub(samples_tail_len);
        tail_valid     = n_total - tail_start;

        let mut win_i:     Option<usize> = None;
        let mut win_mag:   i32           = 0;
        let mut win_phase: i32           = -1;

        let mut i = 0;
        while i < n_total.saturating_sub(samples_tail_len) {
            if !(work_buf[i + 1]  > work_buf[i + 7]
              && work_buf[i + 12] > work_buf[i + 14]
              && work_buf[i + 12] > work_buf[i + 15])
            {
                i += 1;
                continue;
            }

            let (_noise, ref_level) =
                compute_noise_and_threshold(&work_buf, i, threshold_factor);

            let mut best_phase: i32 = -1;
            let mut best_mag:   i32 = 0;

            for phase in 0..5 {
                let pa_mag = compute_preamble_magnitude(&work_buf, i, phase as u16);
                if pa_mag > best_mag && pa_mag > ref_level as i32 {
                    best_mag   = pa_mag;
                    best_phase = phase as i32;
                }
            }

            if best_phase >= 0 {
                if config.use_nms {
                    // NMS: track strongest candidate in sliding window
                    if let Some(prev_i) = win_i {
                        if i - prev_i <= nms_window {
                            if best_mag > win_mag {
                                win_i     = Some(i);
                                win_mag   = best_mag;
                                win_phase = best_phase;
                            }
                        } else {
                            if tx
                                .send(ADSBCandidate {
                                    data:       work_buf[prev_i..prev_i + 300].to_vec(),
                                    best_phase: win_phase as u8,
                                })
                                .is_err()
                            {
                                break;
                            }
                            win_i     = Some(i);
                            win_mag   = best_mag;
                            win_phase = best_phase;
                        }
                    } else {
                        win_i     = Some(i);
                        win_mag   = best_mag;
                        win_phase = best_phase;
                    }
                } else {
                    // Blind skip: emit immediately, advance past this candidate
                    if tx
                        .send(ADSBCandidate {
                            data:       work_buf[i..i + 300].to_vec(),
                            best_phase: best_phase as u8,
                        })
                        .is_err()
                    {
                        break;
                    }
                    i += nms_window; // extra advance; loop adds 1 more below
                }
            }

            i += 1;
        }

        work_buf.copy_within(tail_start..n_total, 0);

        // Flush remaining NMS window at chunk boundary
        if config.use_nms {
            if let Some(prev_i) = win_i {
                if tx
                    .send(ADSBCandidate {
                        data:       work_buf[prev_i..prev_i + 300].to_vec(),
                        best_phase: win_phase as u8,
                    })
                    .is_err()
                {
                    break;
                }
            }
        }
    }
}

struct ADSBMessage {
    bytes:          [u8; 14],
    len:            u8,
    preamble_phase: u8,
    decode_phase:   u8,
}

fn spiral_from(start: u8, n: u8) -> Vec<usize> {
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

struct DecodeResult {
    df:    u8,
    bytes: Vec<u8>,
    len:   u8,
}

fn decode_bit_correlation(samples: &[u16], sample_offset: usize, subsample_phase: u8) -> u8 {
    let window = &samples[sample_offset..];
    if window.len() < 3 { return 0; }

    let s0 = window[0] as i32;
    let s1 = window[1] as i32;
    let s2 = window[2] as i32;

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

fn decode_bits(samples: &[u16], start_offset: usize, phase: u8, num_bits: u8) -> Vec<u8> {
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

const VALID_DF_SHORT: [u8; 4] = [0, 4, 5, 11];
const VALID_DF_LONG:  [u8; 7] = [16, 17, 18, 19, 20, 21, 24];

fn try_decoding(samples: &[u16], trial_phase: u8) -> Option<DecodeResult> {
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
    let mut msg      = vec![0u8; num_bytes as usize];
    for i in 0..num_bytes as usize {
        for bit in &payload_bits[i * 8..(i + 1) * 8] {
            msg[i] = (msg[i] << 1) | bit;
        }
    }
    Some(DecodeResult { df, bytes: msg, len: num_bytes })
}

const POLY: u32 = 0xFFF409;

fn modes_checksum(msg: &[u8]) -> u32 {
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

fn decoding(
    _config: Arc<ADSBConfig>,
    rx: crossbeam_channel::Receiver<ADSBCandidate>,
    tx: crossbeam_channel::Sender<ADSBMessage>,
) {
    let mut icao_filter: HashSet<u32> = HashSet::new();

    while let Ok(adsb_candidate) = rx.recv() {
        let samples        = adsb_candidate.data;
        let preamble_phase = adsb_candidate.best_phase;
        let mut best_hex:    Option<Vec<u8>> = None;
        let mut decode_phase: u8             = preamble_phase;

        for phase in spiral_from(preamble_phase, 5) {
            let Some(result) = try_decoding(&samples, phase as u8) else { continue };
            let df  = result.df;
            let msg = result.bytes;
            let len = result.len;
            let crc = modes_checksum(&msg[..len as usize]);

            if df == 17 || df == 18 {
                if crc != 0 { continue; }
                let icao = (msg[1] as u32) << 16 | (msg[2] as u32) << 8 | msg[3] as u32;
                icao_filter.insert(icao);
            } else if df == 11 {
                if crc & 0xFFFF80 != 0 { continue; }
                let icao = (msg[1] as u32) << 16 | (msg[2] as u32) << 8 | msg[3] as u32;
                icao_filter.insert(icao);
            } else if matches!(df, 0 | 4 | 5 | 16 | 20 | 21 | 24) {
                if !icao_filter.contains(&crc) { continue; }
            } else {
                continue;
            }

            best_hex     = Some(msg);
            decode_phase = phase as u8;
            break;
        }

        if let Some(msg_bytes) = best_hex {
            let mut bytes = [0u8; 14];
            let len       = msg_bytes.len().min(14);
            bytes[..len].copy_from_slice(&msg_bytes[..len]);
            if tx
                .send(ADSBMessage {
                    bytes,
                    len: len as u8,
                    preamble_phase,
                    decode_phase,
                })
                .is_err()
            {
                break;
            }
        }
    }
}

fn output(config: Arc<ADSBConfig>, rx: crossbeam_channel::Receiver<ADSBMessage>) {
    while let Ok(msg) = rx.recv() {
        let hex: String = msg.bytes[..msg.len as usize]
            .iter()
            .map(|b| format!("{:02x}", b))
            .collect();
        if config.debug {
            let delta     = msg.decode_phase as i8 - msg.preamble_phase as i8;
            let match_str = if delta == 0 {
                "exact".to_string()
            } else {
                format!("delta={:+}", delta)
            };
            println!(
                "{hex}  [preamble_phase={} decode_phase={} {}]",
                msg.preamble_phase, msg.decode_phase, match_str
            );
        } else {
            println!("{hex}");
        }
    }
}

fn main() {
    let file_path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| String::from("/home/louis/Documents/ADS-B/data/modes1_2.4mhz.bin"));

    let config = Arc::new(ADSBConfig { file_path, ..ADSBConfig::default() });

    let reader_config   = Arc::clone(&config);
    let detector_config = Arc::clone(&config);
    let decoder_config  = Arc::clone(&config);
    let output_config   = Arc::clone(&config);

    let (chunk_tx,     chunk_rx)     = bounded::<SampleChunk>(4);
    let (candidate_tx, candidate_rx) = bounded::<ADSBCandidate>(1024);
    let (decode_tx,    decode_rx)    = bounded::<ADSBMessage>(1024);

    let reader = if config.realtime {
        #[cfg(feature = "sdr")]
        { thread::spawn(move || soapy_streaming(reader_config, chunk_tx)) }
        #[cfg(not(feature = "sdr"))]
        { panic!("realtime=true requires the 'sdr' feature: cargo run --features sdr") }
    } else {
        thread::spawn(move || read_file(reader_config, chunk_tx).unwrap())
    };
    let preamble_detector =
        thread::spawn(move || preamble_detection(detector_config, chunk_rx, candidate_tx));
    let decoder =
        thread::spawn(move || decoding(decoder_config, candidate_rx, decode_tx));
    let output_thread =
        thread::spawn(move || output(output_config, decode_rx));

    reader.join().unwrap();
    preamble_detector.join().unwrap();
    decoder.join().unwrap();
    output_thread.join().unwrap();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_spiral_from_center() {
        assert_eq!(spiral_from(2, 5), vec![2, 1, 3, 0, 4]);
    }

    #[test]
    fn test_spiral_from_zero() {
        // Starting at 0: only +deltas available
        assert_eq!(spiral_from(0, 5), vec![0, 1, 2, 3, 4]);
    }

    #[test]
    fn test_modes_checksum_df17() {
        // DF17 from readsb_output.txt: 8f4d2023587f345e35837e2218b2 — CRC must be 0
        let msg: Vec<u8> = vec![
            0x8f, 0x4d, 0x20, 0x23, 0x58, 0x7f, 0x34, 0x5e,
            0x35, 0x83, 0x7e, 0x22, 0x18, 0xb2,
        ];
        assert_eq!(modes_checksum(&msg), 0);
    }

    #[test]
    fn test_modes_checksum_df11() {
        // DF11 from readsb_output.txt: 5d4d20237a55af — top 17 bits of CRC must be 0
        let msg: Vec<u8> = vec![0x5d, 0x4d, 0x20, 0x23, 0x7a, 0x55, 0xaf];
        let crc = modes_checksum(&msg);
        assert_eq!(
            crc & 0xFFFF80, 0,
            "DF11 CRC top 17 bits must be 0, got {:#08x}", crc
        );
    }

    #[test]
    fn test_noise_threshold() {
        // Place known values at the five noise sample positions (idx + 5, 8, 16, 17, 18)
        let mut samples = vec![0u16; 30];
        samples[5]  = 100;
        samples[8]  = 200;
        samples[16] = 300;
        samples[17] = 400;
        samples[18] = 500;
        let (noise, ref_level) = compute_noise_and_threshold(&samples, 0, 50);
        assert_eq!(noise, 1500);
        // ref_level = (1500 * 50) >> 5 = 75000 >> 5 = 2343
        assert_eq!(ref_level, 2343);
    }

    #[test]
    fn test_integration_file_decode() {
        let path = "/home/louis/Documents/ADS-B/data/modes1_2.4mhz.bin";
        if !std::path::Path::new(path).exists() {
            println!("skipping: test file not found at {}", path);
            return;
        }

        let config = Arc::new(ADSBConfig {
            file_path: path.to_string(),
            ..ADSBConfig::default()
        });

        let reader_config   = Arc::clone(&config);
        let detector_config = Arc::clone(&config);
        let decoder_config  = Arc::clone(&config);

        let (chunk_tx,     chunk_rx)     = bounded::<SampleChunk>(4);
        let (candidate_tx, candidate_rx) = bounded::<ADSBCandidate>(1024);
        let (decode_tx,    decode_rx)    = bounded::<ADSBMessage>(1024);

        let reader   = thread::spawn(move || read_file(reader_config, chunk_tx).unwrap());
        let detector = thread::spawn(move || preamble_detection(detector_config, chunk_rx, candidate_tx));
        let decoder  = thread::spawn(move || decoding(decoder_config, candidate_rx, decode_tx));

        reader.join().unwrap();
        detector.join().unwrap();
        decoder.join().unwrap();

        let mut unique: HashSet<Vec<u8>> = HashSet::new();
        while let Ok(msg) = decode_rx.try_recv() {
            unique.insert(msg.bytes[..msg.len as usize].to_vec());
        }
        assert!(
            unique.len() >= 160,
            "Expected >= 160 unique messages, got {}",
            unique.len()
        );
    }

}
