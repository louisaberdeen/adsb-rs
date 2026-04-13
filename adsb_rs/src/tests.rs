use super::*;
use crossbeam_channel::bounded;

// ─── nl() ────────────────────────────────────────────────────────────────────

#[test]
fn test_nl_equator() {
    // At the equator, 59 longitude zones (maximum)
    assert_eq!(nl(0.0), 59);
}

#[test]
fn test_nl_symmetric() {
    // nl() takes abs(); southern hemisphere mirrors northern
    assert_eq!(nl(-37.0), nl(37.0));
    assert_eq!(nl(-51.5), nl(51.5));
}

#[test]
fn test_nl_malta() {
    // AMC421 decoded at ~37°N near Malta.
    // The NL bug (off-by-one table) returned 46 here; correct value is 47.
    // 37.0 < 38.41241892 → 47
    assert_eq!(nl(37.0), 47);
}

#[test]
fn test_nl_london() {
    // London Heathrow ~51.5°N: 51.5 < 51.89342469 → 37
    assert_eq!(nl(51.5), 37);
}

#[test]
fn test_nl_pole() {
    // At and above 87°: falls through all thresholds → 1
    assert_eq!(nl(87.0), 1);
    assert_eq!(nl(90.0), 1);
}

#[test]
fn test_nl_near_pole() {
    // 86.53536998 ≤ 86.6 < 87.0 → 2
    assert_eq!(nl(86.6), 2);
}

// ─── decode_callsign() ───────────────────────────────────────────────────────

#[test]
fn test_decode_callsign_basic() {
    // "ABCDEFGH": AIS values 1–8, no trailing spaces.
    // me[0] = TC byte (ignored); me[1..7] = packed characters.
    // Bit layout: 000001|000010|000011|000100|000101|000110|000111|001000
    // Bytes:       0x04    0x20   0xC4   0x14   0x61   0xC8
    let me: Vec<u8> = vec![0x20, 0x04, 0x20, 0xC4, 0x14, 0x61, 0xC8];
    assert_eq!(decode_callsign(&me), "ABCDEFGH");
}

#[test]
fn test_decode_callsign_trailing_spaces_trimmed() {
    // "A      " (A + 7 spaces) → trimmed to "A".
    // A=1=000001, space=32=100000
    // Bit layout: 000001|100000|100000|100000|100000|100000|100000|100000
    // Bytes:       0x06    0x08   0x20   0x82   0x08   0x20
    let me: Vec<u8> = vec![0x08, 0x06, 0x08, 0x20, 0x82, 0x08, 0x20];
    assert_eq!(decode_callsign(&me), "A");
}

// ─── decode_altitude() ───────────────────────────────────────────────────────

#[test]
fn test_decode_altitude_25000ft() {
    // alt_raw = 0x830, q_bit = 1, n = 1040 → 1040 * 25 − 1000 = 25000 ft
    // me[1] = alt_raw >> 4 = 0x83, me[2] top nibble = alt_raw & 0xF = 0
    let me: Vec<u8> = vec![0x90, 0x83, 0x00, 0x00, 0x00, 0x00, 0x00];
    assert_eq!(decode_altitude(&me), Some(25000));
}

#[test]
fn test_decode_altitude_sea_level() {
    // alt_raw = 0x058, q_bit = 1, n = 40 → 40 * 25 − 1000 = 0 ft
    // me[1] = 0x058 >> 4 = 0x05, me[2] top nibble = 0x058 & 0xF = 8 → 0x80
    let me: Vec<u8> = vec![0x90, 0x05, 0x80, 0x00, 0x00, 0x00, 0x00];
    assert_eq!(decode_altitude(&me), Some(0));
}

// ─── decode_cpr() ────────────────────────────────────────────────────────────

#[test]
fn test_decode_cpr_even() {
    // F = 0 (even frame), lat17 = 0x12345, lon17 = 0x6789
    // me_u64 = (0x12345 << 17) | 0x6789 = 0x0000_0002_468A_6789
    // → bytes [0, me[0], me[1]…me[6]] = [0, 0x00, 0x00, 0x02, 0x46, 0x8A, 0x67, 0x89]
    let me: Vec<u8> = vec![0x00, 0x00, 0x02, 0x46, 0x8A, 0x67, 0x89];
    let (is_odd, lat17, lon17) = decode_cpr(&me);
    assert!(!is_odd);
    assert_eq!(lat17, 0x12345);
    assert_eq!(lon17, 0x6789);
}

#[test]
fn test_decode_cpr_odd() {
    // F = 1 (odd frame): set bit 34 of me_u64 → adds 0x4_0000_0000 → me[2] += 0x04
    let me: Vec<u8> = vec![0x00, 0x00, 0x06, 0x46, 0x8A, 0x67, 0x89];
    let (is_odd, lat17, lon17) = decode_cpr(&me);
    assert!(is_odd);
    assert_eq!(lat17, 0x12345);
    assert_eq!(lon17, 0x6789);
}

// ─── cpr_global_decode() ─────────────────────────────────────────────────────

#[test]
fn test_cpr_global_decode_paris() {
    // Position: lat ≈ 48.880°N, lon ≈ 2.301°E (Paris area)
    //
    // Derived analytically using the CPR algorithm:
    //   dlat_e = 6.0,  zone_j = 8,  lat_frac_e = 0.14671 → lat17_e = 19219
    //   dlat_o = 360/59, zone_j = 8, lat_frac_o = 0.01088 → lat17_o = 1426
    //   nl(48.880) = 39, dlon_e = 360/39,  lon_frac_e = 0.24941 → lon17_e = 32678
    //                    dlon_o = 360/38,  lon_frac_o = 0.24306 → lon17_o = 31847
    let now   = std::time::Instant::now();
    let even  = CprFrame { lat17: 19219, lon17: 32678, received_at: now };
    let odd   = CprFrame { lat17: 1426,  lon17: 31847, received_at: now };

    let result = cpr_global_decode(&even, &odd, false); // even frame is most recent
    assert!(result.is_some(), "CPR decode returned None");
    let (lat, lon) = result.unwrap();
    assert!((lat - 48.880).abs() < 0.01, "lat = {lat:.4}, expected ~48.880");
    assert!((lon -  2.301).abs() < 0.01, "lon = {lon:.4}, expected ~2.301");
}

// ─── decode_velocity() ───────────────────────────────────────────────────────

#[test]
fn test_decode_velocity_east_north() {
    // EW = 300 kts east (dir_ew = 0), NS = 400 kts north (dir_ns = 0)
    // → GS = 500 kts, track ≈ 36.87° (NE), baro_rate = 0
    //
    // b = (301 << 32) | (401 << 21)  with me[0] = 0x99 (TC19, subtype 1)
    // b = 0x0099_012D_3220_0000
    // → [me[0]…me[6]] = [0x99, 0x01, 0x2D, 0x32, 0x20, 0x00, 0x00]
    let me: Vec<u8> = vec![0x99, 0x01, 0x2D, 0x32, 0x20, 0x00, 0x00];
    let result = decode_velocity(&me);
    assert!(result.is_some(), "decode_velocity returned None");
    let (gs, track, vert) = result.unwrap();
    assert!((gs    - 500.0).abs() < 0.1,  "gs = {gs:.2}, expected 500.0");
    assert!((track -  36.87).abs() < 0.1, "track = {track:.2}, expected ~36.87°");
    assert_eq!(vert, 0);
}

// ─── spiral_from() ───────────────────────────────────────────────────────────

#[test]
fn test_spiral_from_center() {
    assert_eq!(spiral_from(2, 5), vec![2, 1, 3, 0, 4]);
}

#[test]
fn test_spiral_from_zero() {
    // Starting at 0: only +deltas available
    assert_eq!(spiral_from(0, 5), vec![0, 1, 2, 3, 4]);
}

// ─── modes_checksum() ────────────────────────────────────────────────────────

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
    assert_eq!(crc & 0xFFFF80, 0, "DF11 top 17 CRC bits must be 0, got {crc:#08x}");
}

// ─── compute_noise_and_threshold() ───────────────────────────────────────────

#[test]
fn test_noise_threshold() {
    // Known values at the five mandatory-quiet sample positions (offsets 5, 8, 16, 17, 18)
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

// ─── integration: full pipeline ──────────────────────────────────────────────

#[test]
fn test_integration_file_decode() {
    let path = "/home/louis/Documents/ADS-B/data/modes1_2.4mhz.bin";
    if !std::path::Path::new(path).exists() {
        println!("skipping: test file not found at {path}");
        return;
    }

    let config = Arc::new(ADSBConfig { file_path: path.to_string(), ..ADSBConfig::default() });

    let (chunk_tx,     chunk_rx)     = bounded::<SampleChunk>(4);
    let (candidate_tx, candidate_rx) = bounded::<ADSBCandidate>(1024);
    let (decode_tx,    decode_rx)    = bounded::<ADSBMessage>(1024);

    let cfg = Arc::clone(&config);
    let reader   = thread::spawn(move || read_file(cfg, chunk_tx).unwrap());
    let cfg = Arc::clone(&config);
    let detector = thread::spawn(move || preamble_detection(cfg, chunk_rx, candidate_tx));
    let cfg = Arc::clone(&config);
    let decoder  = thread::spawn(move || decoding(cfg, candidate_rx, decode_tx));

    reader.join().unwrap();
    detector.join().unwrap();
    decoder.join().unwrap();

    let mut unique: HashSet<Vec<u8>> = HashSet::new();
    while let Ok(msg) = decode_rx.try_recv() {
        unique.insert(msg.bytes[..msg.len as usize].to_vec());
    }
    assert!(unique.len() >= 160, "Expected ≥ 160 unique messages, got {}", unique.len());
}

#[test]
fn test_integration_json_output() {
    let path = "/home/louis/Documents/ADS-B/data/modes1_2.4mhz.bin";
    if !std::path::Path::new(path).exists() {
        println!("skipping: test file not found at {path}");
        return;
    }

    let dir = "/tmp/adsb_rs_test_json";
    std::fs::create_dir_all(dir).unwrap();

    let config = Arc::new(ADSBConfig {
        file_path: path.to_string(),
        json_dir:  Some(dir.to_string()),
        ..ADSBConfig::default()
    });

    let (chunk_tx,     chunk_rx)     = bounded::<SampleChunk>(4);
    let (candidate_tx, candidate_rx) = bounded::<ADSBCandidate>(1024);
    let (decode_tx,    decode_rx)    = bounded::<ADSBMessage>(1024);
    let (state_tx,     state_rx)     = bounded::<ADSBMessage>(1024);

    let cfg = Arc::clone(&config);
    let reader   = thread::spawn(move || read_file(cfg, chunk_tx).unwrap());
    let cfg = Arc::clone(&config);
    let detector = thread::spawn(move || preamble_detection(cfg, chunk_rx, candidate_tx));
    let cfg = Arc::clone(&config);
    let decoder  = thread::spawn(move || decoding(cfg, candidate_rx, decode_tx));
    let tracker  = thread::spawn(move || state_tracker(Arc::clone(&config), decode_rx, state_tx));

    // Drain the output side so state_tracker is never blocked on send
    while state_rx.recv().is_ok() {}

    reader.join().unwrap();
    detector.join().unwrap();
    decoder.join().unwrap();
    tracker.join().unwrap();

    let json_path = format!("{dir}/aircraft.json");
    let content   = std::fs::read_to_string(&json_path)
        .expect("aircraft.json was not written by state_tracker");
    let v: serde_json::Value = serde_json::from_str(&content)
        .expect("aircraft.json is not valid JSON");

    let aircraft = v["aircraft"].as_array().expect("missing 'aircraft' array");
    assert!(!aircraft.is_empty(), "no aircraft in aircraft.json");

    // Every entry must have a 6-character lowercase hex ICAO address
    assert!(
        aircraft.iter().all(|a| a["hex"].as_str().map_or(false, |h| h.len() == 6)),
        "one or more aircraft missing a valid 6-char hex field"
    );

    std::fs::remove_dir_all(dir).ok();
}
