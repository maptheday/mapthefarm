#!/usr/bin/env python3
"""
Shared helper for the ON-ESP sims.

The whole simulation runs on the ESP (the `sim` firmware build: QuadSim physics
inside the real flight loop). The drone flies autonomously from boot and logs the
flight to LittleFS. This module just: opens the port (which resets the ESP and
restarts the flight), waits for it to land, asks for the log with the DUMPLOG
serial command, and parses it into viz-ready samples.

Each on-esp "scenario" (see the files next to this one) is a thin wrapper that
calls fly_and_pull(), asserts what it expects, and saves the JSON for the map.
"""
import json, math, time
import serial   # pyserial (present in simulate/.venv-rotorpy)

# The farmer's real field: 8 captured GPS corners, corner 1 = launch/home.
# Must match WAYPOINTS in FlightConfig.hpp and OnboardSim's home.
CORNERS_LATLON = [
    (35.948305, -78.241377),  # 1 (launch/home)
    (35.947693, -78.240760),  # 2
    (35.947806, -78.239370),  # 3
    (35.947923, -78.239510),  # 4
    (35.948327, -78.238051),  # 5
    (35.949061, -78.238292),  # 6
    (35.948587, -78.240529),  # 7
    (35.948500, -78.240432),  # 8
]
HOME_LAT, HOME_LON = CORNERS_LATLON[0]
M_PER_DEG = 111320.0
GEOFENCE_M = 450.0
FT_PER_M = 3.28084


def corner_en(lat, lon):
    """A lat/lon -> (east_m, north_m) relative to home."""
    return ((lon - HOME_LON) * M_PER_DEG * math.cos(math.radians(HOME_LAT)),
            (lat - HOME_LAT) * M_PER_DEG)


def fly_and_pull(port, scenario="full", wait=320.0, verbose=True):
    """Reset the ESP, pick the scenario, wait for it to finish, pull the log.
    Returns a viz-ready dict: {home, geofence_m, corners, samples}."""
    ser = serial.Serial(port, 115200, timeout=0.2)
    # Native USB-CDC does NOT auto-reset on port open, so pulse DTR/RTS (the same
    # lines esptool uses) to reboot the ESP -- otherwise a second scenario would
    # attach to the already-landed drone and pull a stale log.
    try:
        ser.setDTR(False); ser.setRTS(True); time.sleep(0.12)   # EN low: hold in reset
        ser.setRTS(False)                                        # release: boot
        ser.setDTR(False)
    except Exception:
        pass
    time.sleep(0.4); ser.reset_input_buffer()
    if verbose:
        print(f"[esp-sim] connected {port} (reset -> fresh flight)")

    # Send the scenario repeatedly through boot + the pre-takeoff window so it
    # lands before the drone arms (~2.5 s after boot).
    done = False
    t_end = time.monotonic() + wait
    sel_until = time.monotonic() + 3.5
    if verbose:
        print(f"[esp-sim] scenario '{scenario}'; waiting up to {wait:.0f}s to finish...")
    while time.monotonic() < t_end:
        if time.monotonic() < sel_until:
            ser.write(f"SCENARIO:{scenario}\n".encode())
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            continue
        if verbose and any(k in line for k in ("NAV", "LAND", "Mission", "SAFETY", "SCENARIO")):
            print("   " + line)
        if "LANDED" in line or "SCENARIO_DONE" in line:
            done = True
            time.sleep(1.0)
            break
    if verbose:
        print(f"[esp-sim] {'finished' if done else 'wait elapsed'} -- requesting log")

    ser.reset_input_buffer()
    ser.write(b"DUMPLOG\n")
    rows, capturing = [], False
    t_end = time.monotonic() + 20
    while time.monotonic() < t_end:
        line = ser.readline().decode(errors="ignore").rstrip("\r\n")
        if line == "===ONBOARD_LOG_START===":
            capturing = True
            continue
        if line == "===ONBOARD_LOG_END===":
            break
        if capturing:
            rows.append(line)
    ser.close()

    samples = []
    for r in rows:
        if not r or r.startswith("#") or r.startswith("t_s"):
            continue
        f = r.split(",")
        if len(f) < 9:
            continue
        t, phase, north, east, up_ft, roll, pitch, dist, wp = f[:9]
        samples.append({"t": float(t), "phase": phase,
                        "north": float(north), "east": float(east),
                        "up": round(float(up_ft) / FT_PER_M, 1),   # meters
                        "roll": float(roll), "pitch": float(pitch),
                        "dist": float(dist), "wp": int(wp), "heading": 0.0})

    corners = [{"n": n, "e": e} for (e, n) in (corner_en(la, lo) for (la, lo) in CORNERS_LATLON)]
    return {"home": {"lat": HOME_LAT, "lon": HOME_LON}, "geofence_m": GEOFENCE_M,
            "corners": corners, "vehicle": "onboard", "samples": samples}


def save(data, path):
    with open(path, "w") as fp:
        json.dump(data, fp)
    print(f"[esp-sim] wrote {len(data['samples'])} samples -> {path}")
