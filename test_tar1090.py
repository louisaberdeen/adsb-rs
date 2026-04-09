#!/usr/bin/env python3
"""
Write a fake aircraft.json to /tmp/adsb_json every second so you can verify
tar1090 is displaying data correctly before connecting adsb_rs.

Open http://localhost/tar1090 and you should see a plane over London.

Usage:
    python3 test_tar1090.py [--dir /tmp/adsb_json]

Press Ctrl+C to stop.
"""
import json, time, math, argparse, sys
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--dir", default="/tmp/adsb_json")
args = parser.parse_args()

out_dir = Path(args.dir)
out_dir.mkdir(parents=True, exist_ok=True)

# Fake aircraft: over London, flying north, descending
ICAO     = "4ca7ed"
CALLSIGN = "TEST001 "
LAT_BASE = 51.5074   # London
LON_BASE = -0.1278
ALTITUDE = 15000     # feet

print(f"Writing fake aircraft to {out_dir}/aircraft.json every second")
print(f"Open: http://localhost/tar1090")
print(f"You should see aircraft {ICAO} ({CALLSIGN.strip()}) near London")
print("Ctrl+C to stop\n")

msg_count = 0
try:
    while True:
        now = time.time()
        # Slowly drift the position so the track is visible
        elapsed = now % 600          # cycle every 10 min
        lat = LAT_BASE + elapsed * 0.0005
        lon = LON_BASE + elapsed * 0.0002

        msg_count += 100
        aircraft = {
            "now":      now,
            "messages": msg_count,
            "aircraft": [
                {
                    "hex":      ICAO,
                    "flight":   CALLSIGN,
                    "lat":      round(lat, 6),
                    "lon":      round(lon, 6),
                    "alt_baro": ALTITUDE,
                    "gs":       420.0,
                    "track":    15.0,
                    "baro_rate": -256,
                    "seen":     0.5,
                    "seen_pos": 0.5,
                    "messages": msg_count,
                }
            ]
        }

        tmp  = out_dir / "aircraft.json.tmp"
        final = out_dir / "aircraft.json"
        tmp.write_text(json.dumps(aircraft))
        tmp.rename(final)

        print(f"\r  {time.strftime('%H:%M:%S')}  lat={lat:.5f}  lon={lon:.5f}  alt={ALTITUDE} ft  msgs={msg_count}", end="", flush=True)
        time.sleep(1)

except KeyboardInterrupt:
    print("\nStopped.")
