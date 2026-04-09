use std::collections::{HashMap, HashSet};
use std::fs::File;
use std::io::{self, BufReader, Read, Write};
use std::net::TcpListener;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use crossbeam_channel::RecvTimeoutError;
use serde::Serialize;

use clap::Parser;
use crossbeam_channel::bounded;

#[derive(Parser)]
#[command(name = "adsb_rs", about = "ADS-B decoder — file or live SDR → hex Mode S messages")]
struct Cli {
    /// UC8 IQ sample file to decode (2.4 MHz sample rate)
    file: Option<String>,

    /// Stream live from a connected SDR (requires: cargo build --features sdr)
    #[arg(short, long, conflicts_with = "file")]
    live: bool,

    /// SoapySDR device args, e.g. "driver=rtlsdr" (default: auto-detect first device)
    #[arg(short, long, default_value = "")]
    device: String,

    /// Gain in dB; omit for auto AGC
    #[arg(short, long)]
    gain: Option<f64>,

    /// Sample rate in Hz
    #[arg(short = 'r', long, default_value_t = 2_400_000.0)]
    rate: f64,

    /// Centre frequency in Hz
    #[arg(short = 'f', long, default_value_t = 1_090_000_000.0)]
    freq: f64,

    /// Print preamble/decode phase info alongside each hex message
    #[arg(long)]
    debug: bool,

    /// Use blind skip instead of non-maximum suppression
    #[arg(long)]
    no_nms: bool,

    /// Loop file input continuously (useful with --serve for persistent testing)
    #[arg(long)]
    r#loop: bool,

    /// List available SoapySDR devices and exit (requires --features sdr)
    #[arg(long)]
    list_devices: bool,

    /// Serve decoded messages as an AVR TCP stream on PORT (compatible with readsb/tar1090)
    #[arg(long, value_name = "PORT")]
    serve: Option<u16>,

    /// Write aircraft.json to DIR every second for tar1090 (no readsb required)
    #[arg(long, value_name = "DIR")]
    json: Option<String>,
}

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
    serve_port:       Option<u16>,
    loop_file:        bool,
    json_dir:         Option<String>,
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
            serve_port:       None,
            loop_file:        false,
            json_dir:         None,
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
    use std::io::Seek;

    let file = File::open(&config.file_path)?;
    let mut reader = BufReader::new(file);

    let mut chunk      = vec![0u8;  config.chunk_size];
    let mut magnitudes = vec![0u16; config.chunk_size / 2];

    loop {
        let n = reader.read(&mut chunk)?;
        if n == 0 {
            if config.loop_file {
                reader.seek(io::SeekFrom::Start(0))?;
                continue;
            }
            break;
        }

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

// =============================================================================
// State tracking and aircraft.json generation
// =============================================================================

struct CprFrame {
    lat17:       u32,
    lon17:       u32,
    received_at: Instant,
}

struct AircraftState {
    callsign:      Option<String>,
    altitude_baro: Option<i32>,
    altitude_geom: Option<i32>,
    latitude:      Option<f64>,
    longitude:     Option<f64>,
    ground_speed:  Option<f64>,
    track:         Option<f64>,
    vert_rate:     Option<i32>,
    cpr_even:      Option<CprFrame>,
    cpr_odd:       Option<CprFrame>,
    last_seen:     Instant,
    last_position: Option<Instant>,
    message_count: u32,
}

impl AircraftState {
    fn new(now: Instant) -> Self {
        Self {
            callsign:      None,
            altitude_baro: None,
            altitude_geom: None,
            latitude:      None,
            longitude:     None,
            ground_speed:  None,
            track:         None,
            vert_rate:     None,
            cpr_even:      None,
            cpr_odd:       None,
            last_seen:     now,
            last_position: None,
            message_count: 0,
        }
    }
}

#[derive(Serialize)]
struct AircraftJson {
    hex:                          String,
    #[serde(skip_serializing_if = "Option::is_none")]
    flight:                       Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    alt_baro:                     Option<i32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    alt_geom:                     Option<i32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    gs:                           Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    track:                        Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    baro_rate:                    Option<i32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    lat:                          Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    lon:                          Option<f64>,
    seen:                         f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    seen_pos:                     Option<f64>,
    messages:                     u32,
}

#[derive(Serialize)]
struct AircraftFileJson {
    now:      f64,
    messages: u64,
    aircraft: Vec<AircraftJson>,
}

#[derive(Serialize)]
struct ReceiverJson {
    version: &'static str,
    refresh: u32,
    history: u32,
}

// NL(lat): number of longitude zones for a given latitude.
// Thresholds from ICAO Annex 10 Appendix B, matching dump1090/readsb exactly.
fn nl(lat: f64) -> u32 {
    let lat = lat.abs();
    if lat < 10.47047130 { return 59; }
    if lat < 14.82817437 { return 58; }
    if lat < 18.18626357 { return 57; }
    if lat < 21.02939493 { return 56; }
    if lat < 23.54504487 { return 55; }
    if lat < 25.82924707 { return 54; }
    if lat < 27.93898710 { return 53; }
    if lat < 29.91135686 { return 52; }
    if lat < 31.77209708 { return 51; }
    if lat < 33.53993436 { return 50; }
    if lat < 35.22899598 { return 49; }
    if lat < 36.85025108 { return 48; }
    if lat < 38.41241892 { return 47; }
    if lat < 39.92256684 { return 46; }
    if lat < 41.38651832 { return 45; }
    if lat < 42.80914012 { return 44; }
    if lat < 44.19454951 { return 43; }
    if lat < 45.54626723 { return 42; }
    if lat < 46.86733252 { return 41; }
    if lat < 48.16039128 { return 40; }
    if lat < 49.42776439 { return 39; }
    if lat < 50.67150166 { return 38; }
    if lat < 51.89342469 { return 37; }
    if lat < 53.09516153 { return 36; }
    if lat < 54.27817472 { return 35; }
    if lat < 55.44378444 { return 34; }
    if lat < 56.59318756 { return 33; }
    if lat < 57.72747354 { return 32; }
    if lat < 58.84763776 { return 31; }
    if lat < 59.95459277 { return 30; }
    if lat < 61.04917774 { return 29; }
    if lat < 62.13216659 { return 28; }
    if lat < 63.20427479 { return 27; }
    if lat < 64.26616523 { return 26; }
    if lat < 65.31845310 { return 25; }
    if lat < 66.36171008 { return 24; }
    if lat < 67.39646774 { return 23; }
    if lat < 68.42322022 { return 22; }
    if lat < 69.44242631 { return 21; }
    if lat < 70.45451075 { return 20; }
    if lat < 71.45986473 { return 19; }
    if lat < 72.45884545 { return 18; }
    if lat < 73.45177442 { return 17; }
    if lat < 74.43893416 { return 16; }
    if lat < 75.42056257 { return 15; }
    if lat < 76.39684391 { return 14; }
    if lat < 77.36789461 { return 13; }
    if lat < 78.33374083 { return 12; }
    if lat < 79.29428225 { return 11; }
    if lat < 80.24923213 { return 10; }
    if lat < 81.19801349 { return  9; }
    if lat < 82.13956981 { return  8; }
    if lat < 83.07199445 { return  7; }
    if lat < 83.99173563 { return  6; }
    if lat < 84.89166191 { return  5; }
    if lat < 85.75541621 { return  4; }
    if lat < 86.53536998 { return  3; }
    if lat < 87.00000000 { return  2; }
    1
}

fn decode_callsign(me: &[u8]) -> String {
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

fn decode_altitude(me: &[u8]) -> Option<i32> {
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

fn decode_cpr(me: &[u8]) -> (bool, u32, u32) {
    // ME bit N (0-indexed from MSB of 56-bit field) sits at u64 bit (55 - N).
    // F flag  = ME bit 21  →  u64 bit 34
    // lat17   = ME bits 22-38  →  u64 bits 33-17  →  (me_u64 >> 17) & 0x1FFFF
    // lon17   = ME bits 39-55  →  u64 bits 16-0   →  me_u64 & 0x1FFFF
    let me_u64 = u64::from_be_bytes([0, me[0], me[1], me[2], me[3], me[4], me[5], me[6]]);
    let f_bit  = ((me_u64 >> 34) & 1) != 0;
    let lat17  = ((me_u64 >> 17) & 0x1FFFF) as u32;
    let lon17  = (me_u64 & 0x1FFFF) as u32;
    (f_bit, lat17, lon17)
}

fn decode_velocity(me: &[u8]) -> Option<(f64, f64, i32)> {
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

fn cpr_global_decode(
    even: &CprFrame,
    odd:  &CprFrame,
    most_recent_is_odd: bool,
) -> Option<(f64, f64)> {
    const NZ: f64 = 15.0;
    let dlat_e = 360.0 / (4.0 * NZ);
    let dlat_o = 360.0 / (4.0 * NZ - 1.0);
    let lat_e  = even.lat17 as f64 / 131072.0;
    let lat_o  = odd.lat17  as f64 / 131072.0;
    let lon_e  = even.lon17 as f64 / 131072.0;
    let lon_o  = odd.lon17  as f64 / 131072.0;

    let j = (59.0 * lat_e - 60.0 * lat_o + 0.5).floor() as i64;
    let mut rlat_e = dlat_e * (j.rem_euclid(60) as f64 + lat_e);
    let mut rlat_o = dlat_o * (j.rem_euclid(59) as f64 + lat_o);
    if rlat_e >= 270.0 { rlat_e -= 360.0; }
    if rlat_o >= 270.0 { rlat_o -= 360.0; }
    if nl(rlat_e) != nl(rlat_o) { return None; }

    let nl_val   = nl(rlat_e);
    let ref_lat  = if most_recent_is_odd { rlat_o } else { rlat_e };
    let ref_lon  = if most_recent_is_odd { lon_o  } else { lon_e  };
    let ni       = (nl_val as i64 - if most_recent_is_odd { 1 } else { 0 }).max(1) as f64;
    let dlon     = 360.0 / ni;
    let m        = (lon_e * (nl_val as f64 - 1.0) - lon_o * nl_val as f64 + 0.5).floor() as i64;
    let mut lon  = dlon * (m.rem_euclid(ni as i64) as f64 + ref_lon);
    if lon >= 180.0  { lon -= 360.0; }
    if lon < -180.0  { lon += 360.0; }
    Some((ref_lat, lon))
}

fn try_cpr_decode(state: &mut AircraftState) {
    let (even, odd) = match (&state.cpr_even, &state.cpr_odd) {
        (Some(e), Some(o)) => (e, o),
        _ => return,
    };
    let newer = even.received_at.max(odd.received_at);
    let older = even.received_at.min(odd.received_at);
    if newer.duration_since(older) > Duration::from_secs(10) { return; }
    let most_recent_is_odd = odd.received_at > even.received_at;
    if let Some((lat, lon)) = cpr_global_decode(even, odd, most_recent_is_odd) {
        state.latitude      = Some(lat);
        state.longitude     = Some(lon);
        state.last_position = Some(Instant::now());
    }
}

fn write_receiver_json(json_dir: &str) {
    let path    = format!("{json_dir}/receiver.json");
    let payload = ReceiverJson { version: env!("CARGO_PKG_VERSION"), refresh: 1000, history: 0 };
    if let Ok(f) = File::create(&path) {
        let _ = serde_json::to_writer(&f, &payload);
    }
}

fn write_aircraft_json(json_dir: &str, states: &HashMap<u32, AircraftState>, total_messages: u64) {
    let now_wall = SystemTime::now()
        .duration_since(UNIX_EPOCH).unwrap_or_default().as_secs_f64();
    let now_inst = Instant::now();

    let aircraft: Vec<AircraftJson> = states.iter().map(|(&icao, s)| {
        AircraftJson {
            hex:       format!("{icao:06x}"),
            flight:    s.callsign.clone(),
            alt_baro:  s.altitude_baro,
            alt_geom:  s.altitude_geom,
            gs:        s.ground_speed,
            track:     s.track,
            baro_rate: s.vert_rate,
            lat:       s.latitude,
            lon:       s.longitude,
            seen:      now_inst.duration_since(s.last_seen).as_secs_f64(),
            seen_pos:  s.last_position.map(|t| now_inst.duration_since(t).as_secs_f64()),
            messages:  s.message_count,
        }
    }).collect();

    let payload  = AircraftFileJson { now: now_wall, messages: total_messages, aircraft };
    let tmp_path = format!("{json_dir}/aircraft.json.tmp");
    let fin_path = format!("{json_dir}/aircraft.json");
    if let Ok(f) = File::create(&tmp_path) {
        if serde_json::to_writer(&f, &payload).is_ok() {
            let _ = std::fs::rename(&tmp_path, &fin_path);
        }
    } else {
        eprintln!("aircraft.json: cannot write to {json_dir}");
    }
}

fn state_tracker(
    json_dir: &str,
    rx:       crossbeam_channel::Receiver<ADSBMessage>,
    tx:       crossbeam_channel::Sender<ADSBMessage>,
) {
    const WRITE_INTERVAL: Duration = Duration::from_secs(1);
    const PURGE_AGE:      Duration = Duration::from_secs(60);

    write_receiver_json(json_dir);

    let mut states:         HashMap<u32, AircraftState> = HashMap::new();
    let mut total_messages: u64                         = 0;
    let mut last_write                                  = Instant::now();

    loop {
        match rx.recv_timeout(Duration::from_millis(100)) {
            Ok(msg) => {
                total_messages += 1;
                let now = Instant::now();
                let df  = msg.bytes[0] >> 3;

                if df == 17 || df == 18 {
                    let icao = (msg.bytes[1] as u32) << 16
                             | (msg.bytes[2] as u32) <<  8
                             |  msg.bytes[3] as u32;
                    let me = &msg.bytes[4..11];
                    let tc = me[0] >> 3;

                    let s = states.entry(icao).or_insert_with(|| AircraftState::new(now));
                    s.last_seen     = now;
                    s.message_count += 1;

                    match tc {
                        1..=4 => {
                            let cs = decode_callsign(me);
                            if !cs.is_empty() { s.callsign = Some(cs); }
                        }
                        9..=18 => {
                            s.altitude_baro = decode_altitude(me);
                            let (is_odd, lat17, lon17) = decode_cpr(me);
                            let frame = CprFrame { lat17, lon17, received_at: now };
                            if is_odd { s.cpr_odd  = Some(frame); }
                            else      { s.cpr_even = Some(frame); }
                            try_cpr_decode(s);
                        }
                        19 => {
                            if let Some((gs, track, vr)) = decode_velocity(me) {
                                s.ground_speed = Some(gs);
                                s.track        = Some(track);
                                s.vert_rate    = Some(vr);
                            }
                        }
                        20..=22 => {
                            s.altitude_geom = decode_altitude(me);
                            let (is_odd, lat17, lon17) = decode_cpr(me);
                            let frame = CprFrame { lat17, lon17, received_at: now };
                            if is_odd { s.cpr_odd  = Some(frame); }
                            else      { s.cpr_even = Some(frame); }
                            try_cpr_decode(s);
                        }
                        _ => {}
                    }
                }

                if tx.send(msg).is_err() { break; }
            }
            Err(RecvTimeoutError::Timeout) => {}
            Err(RecvTimeoutError::Disconnected) => break,
        }

        if last_write.elapsed() >= WRITE_INTERVAL {
            let now = Instant::now();
            states.retain(|_, s| now.duration_since(s.last_seen) < PURGE_AGE);
            write_aircraft_json(json_dir, &states, total_messages);
            last_write = Instant::now();
        }
    }

    // Final flush so short file runs always produce output
    write_aircraft_json(json_dir, &states, total_messages);
}

fn output(
    config:  Arc<ADSBConfig>,
    rx:      crossbeam_channel::Receiver<ADSBMessage>,
    clients: Arc<Mutex<Vec<std::net::TcpStream>>>,
) {
    while let Ok(msg) = rx.recv() {
        let hex: String = msg.bytes[..msg.len as usize]
            .iter()
            .map(|b| format!("{:02x}", b))
            .collect();

        // Broadcast AVR format to any connected TCP clients (readsb/tar1090)
        let avr = format!("*{hex};\n");
        clients.lock().unwrap().retain_mut(|s| s.write_all(avr.as_bytes()).is_ok());

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
    let cli = Cli::parse();

    // --list-devices: enumerate SoapySDR devices and exit
    if cli.list_devices {
        #[cfg(feature = "sdr")]
        {
            let devices = soapysdr::enumerate("").unwrap_or_default();
            if devices.is_empty() {
                eprintln!("No SoapySDR devices found.");
            } else {
                for dev in devices {
                    println!("{}", dev);
                }
            }
            return;
        }
        #[cfg(not(feature = "sdr"))]
        {
            eprintln!("error: --list-devices requires the sdr feature");
            eprintln!("       cargo build --features sdr");
            std::process::exit(1);
        }
    }

    // Validate: need either a file or --live
    if !cli.live && cli.file.is_none() {
        eprintln!("error: provide a FILE to decode or use --live for a connected SDR");
        eprintln!("       Run with --help for usage.");
        std::process::exit(1);
    }

    // --live without the sdr feature compiled in: bail with a clear message
    #[cfg(not(feature = "sdr"))]
    if cli.live {
        eprintln!("error: --live requires the sdr feature");
        eprintln!("       cargo build --features sdr && ./target/release/adsb_rs --live");
        std::process::exit(1);
    }

    let config = Arc::new(ADSBConfig {
        sample_rate:      cli.rate,
        center_freq:      cli.freq,
        gain:             cli.gain,
        sdr_args:         cli.device,
        debug:            cli.debug,
        use_nms:          !cli.no_nms,
        realtime:         cli.live,
        file_path:        cli.file.unwrap_or_default(),
        serve_port:       cli.serve,
        loop_file:        cli.r#loop,
        json_dir:         cli.json,
        ..ADSBConfig::default()
    });

    // Shared list of connected TCP clients for AVR broadcast
    let clients: Arc<Mutex<Vec<std::net::TcpStream>>> = Arc::new(Mutex::new(Vec::new()));

    if let Some(port) = config.serve_port {
        let clients_accept = Arc::clone(&clients);
        thread::spawn(move || {
            let listener = TcpListener::bind(("0.0.0.0", port))
                .unwrap_or_else(|e| { eprintln!("AVR server bind failed: {e}"); std::process::exit(1) });
            eprintln!("AVR server listening on port {port} (connect readsb with --net-connector localhost,{port},raw_in)");
            for stream in listener.incoming().flatten() {
                clients_accept.lock().unwrap().push(stream);
            }
        });
    }

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
        { unreachable!() }
    } else {
        thread::spawn(move || {
            if let Err(e) = read_file(reader_config.clone(), chunk_tx) {
                eprintln!("error reading file '{}': {e}", reader_config.file_path);
                std::process::exit(1);
            }
        })
    };

    let preamble_detector =
        thread::spawn(move || preamble_detection(detector_config, chunk_rx, candidate_tx));
    let decoder =
        thread::spawn(move || decoding(decoder_config, candidate_rx, decode_tx));

    // Optionally insert state_tracker between decoder and output
    let (output_rx, state_tracker_thread) = if let Some(ref dir) = config.json_dir {
        let (state_tx, state_rx) = bounded::<ADSBMessage>(1024);
        let dir = dir.clone();
        let t = thread::spawn(move || state_tracker(&dir, decode_rx, state_tx));
        (state_rx, Some(t))
    } else {
        (decode_rx, None)
    };

    let output_thread =
        thread::spawn(move || output(output_config, output_rx, clients));

    reader.join().unwrap();
    preamble_detector.join().unwrap();
    decoder.join().unwrap();
    if let Some(t) = state_tracker_thread { t.join().unwrap(); }
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
