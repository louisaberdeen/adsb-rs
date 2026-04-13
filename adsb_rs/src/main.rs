mod cpr;
mod decoder;
mod state;

pub use cpr::{CprFrame, nl, decode_cpr, cpr_global_decode};
pub use decoder::{
    spiral_from, DecodeResult, decode_bit_correlation, decode_bits,
    VALID_DF_SHORT, VALID_DF_LONG, POLY, try_decoding, modes_checksum,
    decode_callsign, decode_altitude, decode_velocity,
};
pub use state::state_tracker;
pub use std::collections::HashSet;

use std::collections::HashMap;
use std::fs::File;
use std::io::{self, BufReader, Read, Write};
use std::net::TcpListener;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

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

    /// Suppress hex output; only errors are printed
    #[arg(short = 'q', long)]
    quiet: bool,
}

#[allow(dead_code)]
pub struct ADSBConfig {
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
    quiet:            bool,
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
            file_path:        String::new(),
            serve_port:       None,
            loop_file:        false,
            json_dir:         None,
            quiet:            false,
        }
    }
}

struct SampleChunk {
    data: Vec<u16>,
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
            magnitudes[idx] = (magsq * 65535.0 + 0.5) as u16;
        }
        if tx
            .send(SampleChunk { data: magnitudes[..n / 2].to_vec() })
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

    let mut iq_buf          = vec![Complex::new(0f32, 0f32); num_samples];
    let mut magnitudes      = vec![0u16; num_samples];
    let mut consecutive_err = 0u32;

    loop {
        let n = match stream.read(&mut [iq_buf.as_mut_slice()], 1_000_000) {
            Ok(n)  => { consecutive_err = 0; n }
            Err(e) => {
                consecutive_err += 1;
                eprintln!("SDR read error ({consecutive_err}): {e}");
                if consecutive_err >= 10 {
                    eprintln!("Too many consecutive SDR errors — stopping.");
                    break;
                }
                continue; // overflow / timeout — drop this window and keep going
            }
        };
        for (idx, s) in iq_buf[..n].iter().enumerate() {
            let magsq = (s.re * s.re + s.im * s.im).min(1.0);
            magnitudes[idx] = (magsq * 65535.0 + 0.5) as u16;
        }
        if tx
            .send(SampleChunk { data: magnitudes[..n].to_vec() })
            .is_err()
        {
            break;
        }
    }
    stream.deactivate(None).ok();
}

struct ADSBCandidate {
    data:       [u16; 300], // fixed array — avoids a heap allocation per preamble hit
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
        let chunk_len = sample_chunk.data.len();
        let n_total   = tail_valid + chunk_len;
        work_buf[tail_valid..n_total].copy_from_slice(&sample_chunk.data);

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
                                    data:       work_buf[prev_i..prev_i + 300].try_into().unwrap(),
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
                            data:       work_buf[i..i + 300].try_into().unwrap(),
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
                        data:       work_buf[prev_i..prev_i + 300].try_into().unwrap(),
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

pub struct ADSBMessage {
    pub bytes:          [u8; 14],
    pub len:            u8,
    pub preamble_phase: u8,
    pub decode_phase:   u8,
}

fn decoding(
    _config: Arc<ADSBConfig>,
    rx: crossbeam_channel::Receiver<ADSBCandidate>,
    tx: crossbeam_channel::Sender<ADSBMessage>,
) {
    let mut icao_filter: HashMap<u32, Instant> = HashMap::new();
    const ICAO_TTL:            Duration = Duration::from_secs(3600);
    const ICAO_EVICT_INTERVAL: Duration = Duration::from_secs(300);
    let mut last_eviction = Instant::now();

    while let Ok(adsb_candidate) = rx.recv() {
        let samples        = adsb_candidate.data;
        let preamble_phase = adsb_candidate.best_phase;
        let mut best_result: Option<DecodeResult> = None;
        let mut decode_phase: u8                  = preamble_phase;

        for phase in spiral_from(preamble_phase, 5) {
            let Some(result) = try_decoding(&samples, phase as u8) else { continue };
            let df  = result.df;
            let crc = modes_checksum(&result.bytes[..result.len as usize]);

            if df == 17 || df == 18 {
                if crc != 0 { continue; }
                let icao = (result.bytes[1] as u32) << 16 | (result.bytes[2] as u32) << 8 | result.bytes[3] as u32;
                icao_filter.insert(icao, Instant::now());
            } else if df == 11 {
                if crc & 0xFFFF80 != 0 { continue; }
                let icao = (result.bytes[1] as u32) << 16 | (result.bytes[2] as u32) << 8 | result.bytes[3] as u32;
                icao_filter.insert(icao, Instant::now());
            } else if matches!(df, 0 | 4 | 5 | 16 | 20 | 21 | 24) {
                if !icao_filter.contains_key(&crc) { continue; }
            } else {
                continue;
            }

            best_result  = Some(result);
            decode_phase = phase as u8;
            break;
        }

        // Periodic eviction of stale ICAO entries (prevents unbounded growth in live mode)
        if last_eviction.elapsed() >= ICAO_EVICT_INTERVAL {
            let cutoff = Instant::now() - ICAO_TTL;
            icao_filter.retain(|_, seen_at| *seen_at > cutoff);
            last_eviction = Instant::now();
        }

        if let Some(result) = best_result {
            if tx
                .send(ADSBMessage {
                    bytes: result.bytes,
                    len:   result.len,
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

        if !config.quiet {
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
        quiet:            cli.quiet,
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

    let (chunk_tx,     chunk_rx)     = bounded::<SampleChunk>(64); // large buffer so USB reader is never stalled by a slow pipeline
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
    let (output_rx, state_tracker_thread) = if config.json_dir.is_some() {
        let (state_tx, state_rx) = bounded::<ADSBMessage>(1024);
        let cfg = Arc::clone(&config);
        let t = thread::spawn(move || state_tracker(cfg, decode_rx, state_tx));
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
mod tests;
