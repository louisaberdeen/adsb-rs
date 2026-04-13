use std::collections::HashMap;
use std::fs::File;
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use crossbeam_channel::RecvTimeoutError;
use serde::Serialize;

use crate::cpr::{CprFrame, cpr_global_decode, decode_cpr};
use crate::decoder::{decode_altitude, decode_callsign, decode_velocity};
use crate::{ADSBConfig, ADSBMessage};

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

pub fn state_tracker(
    config: Arc<ADSBConfig>,
    rx:     crossbeam_channel::Receiver<ADSBMessage>,
    tx:     crossbeam_channel::Sender<ADSBMessage>,
) {
    let json_dir = config.json_dir.as_deref().unwrap(); // safe: only called when json_dir is Some

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
