#!/usr/bin/env python3
"""
Record the geofence -> RTL flight as a path for the 2D map animation.

This drives the REAL firmware exactly like the edge_geofence_breach HIL test:
we script the drone's GPS position (fly north out of the geofence, then walk it
back home) while the firmware makes the real decisions -- it detects the 150 m
breach and sequences RTL_CLIMB -> RTL_RETURN -> RTL_SETTLE -> LANDING -> LANDED,
all on its own. We log position + the live phase every tick to JSON.

(Positions are scripted, same as the HIL test. RotorPy-flown positions diverge
today because horizontal nav isn't stabilized in closed loop -- see the bridge.)

Run (system python3 + pyserial; no venv needed):
  python3 simulate/record_geofence_path.py --port /dev/cu.usbmodem14101 --out geofence.json
"""
import argparse, json, math, re, threading, time
import serial

HOME_LAT, HOME_LON = 36.120000, -80.120000
M_PER_DEG_LAT = 111320.0
M_PER_DEG_LON = 111320.0 * math.cos(math.radians(HOME_LAT))
GEOFENCE_M = 150.0
_STATUS_RE = re.compile(r"\[STATUS\] id=\d+ phase=(?P<phase>[A-Z_]+) ")


def latlon(east_m, north_m):
    return HOME_LAT + north_m / M_PER_DEG_LAT, HOME_LON + east_m / M_PER_DEG_LON


class FW:
    def __init__(self, port, baud=115200):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.lines, self.lock, self._buf, self._stop = [], threading.Lock(), bytearray(), False
        threading.Thread(target=self._rx, daemon=True).start()
        threading.Thread(target=self._autogate, daemon=True).start()

    def _rx(self):
        while not self._stop:
            try:
                d = self.ser.read(self.ser.in_waiting or 1)
            except Exception:
                break
            if not d:
                continue
            self._buf.extend(d)
            while b"\n" in self._buf:
                raw, _, self._buf = self._buf.partition(b"\n")
                with self.lock:
                    self.lines.append(raw.decode(errors="replace").strip("\r"))

    def _autogate(self):
        seen, req = 0, re.compile(r"request=([A-Z_]+)")
        while not self._stop:
            with self.lock:
                new, seen = self.lines[seen:], len(self.lines)
            for ln in new:
                m = req.search(ln)
                if m:
                    self.send(f"ALLOW:{m.group(1)}")
            time.sleep(0.01)

    def send(self, s):
        self.ser.write((s + "\n").encode())

    def wait_for(self, sub, timeout=5):
        end, seen = time.monotonic() + timeout, 0
        while time.monotonic() < end:
            with self.lock:
                for ln in self.lines[seen:]:
                    if sub in ln:
                        return True
                seen = len(self.lines)
            time.sleep(0.005)
        return False

    def phase(self, timeout=0.5):
        with self.lock:
            start = len(self.lines)
        self.send("STATUS?1")
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            with self.lock:
                for ln in self.lines[start:]:
                    m = _STATUS_RE.search(ln)
                    if m:
                        return m.group("phase")
                start = len(self.lines)
            time.sleep(0.005)
        return None

    def close(self):
        self._stop = True
        time.sleep(0.1)
        self.ser.close()


def run(args):
    fw = FW(args.port)
    print(f"[rec] connected {args.port}")
    fw.send("RESET:")
    fw.wait_for("State reset", 5)

    north = 0.0          # meters north of home (east stays 0)
    alt_ft = 0.0
    heading = 0.0
    samples = []
    phase = "PARKED"
    t0 = time.monotonic()

    def tick(hold=0.1):
        """Push current scripted sensors, read phase, record a sample."""
        nonlocal phase
        la, lo = latlon(0.0, north)
        fw.send("FIX:1")
        fw.send(f"LAT:{la:.6f}"); fw.send(f"LON:{lo:.6f}")
        fw.send(f"ALT:{alt_ft:.2f}")
        fw.send(f"HDG:{heading:.1f}")
        p = fw.phase()
        if p:
            phase = p
        samples.append({"t": round(time.monotonic() - t0, 2), "north": round(north, 1),
                        "alt_ft": round(alt_ft, 1), "heading": heading,
                        "phase": phase, "dist": round(abs(north), 1)})
        time.sleep(hold)

    def ramp(field_get, field_set, target, secs, hold_pos=None):
        """Linearly move a scripted value to target over secs, ticking as we go."""
        start = field_get()
        steps = max(1, int(secs / 0.1))
        for i in range(1, steps + 1):
            field_set(start + (target - start) * i / steps)
            tick()

    # --- Arm + take off to ~15 ft at home ---
    tick(); tick()
    fw.send("CRSFSTART:1")
    print("[rec] arming / takeoff")
    def set_alt(v):
        nonlocal alt_ft; alt_ft = v
    ramp(lambda: alt_ft, set_alt, 15.0, 5.0)      # climb; firmware RAISE -> HOLD
    for _ in range(10):                            # settle in HOLD
        tick()

    # --- Fly NORTH out of the geofence (drone "holds" but its GPS moves out) ---
    print("[rec] flying north out to ~200 m (trips 150 m geofence -> RTL)")
    heading = 0.0
    def set_north(v):
        nonlocal north; north = v
    ramp(lambda: north, set_north, 200.0, 8.0)     # crosses 150 m -> firmware forces RTL_CLIMB

    # --- RTL_CLIMB: climb to RTL altitude (60 ft) in place ---
    print("[rec] RTL: climbing to 60 ft")
    ramp(lambda: alt_ft, set_alt, 61.0, 5.0)
    for _ in range(10):
        tick()

    # --- RTL_RETURN: fly back home (GPS walks north -> 0) ---
    print("[rec] RTL: returning home")
    heading = 180.0
    ramp(lambda: north, set_north, 0.0, 8.0)
    for _ in range(10):                            # RTL_RETURN -> RTL_SETTLE (at home)
        tick()

    # --- RTL_SETTLE: hover ~3 s, then LANDING ---
    print("[rec] RTL: settling")
    for _ in range(40):
        tick()

    # --- LANDING: descend to the ground ---
    print("[rec] landing")
    ramp(lambda: alt_ft, set_alt, 0.0, 8.0)
    for _ in range(20):
        tick()

    fw.close()
    out = {"home": {"lat": HOME_LAT, "lon": HOME_LON}, "geofence_m": GEOFENCE_M,
           "samples": samples}
    with open(args.out, "w") as f:
        json.dump(out, f)
    phases = []
    for s in samples:
        if not phases or phases[-1] != s["phase"]:
            phases.append(s["phase"])
    print(f"\n[rec] wrote {len(samples)} samples -> {args.out}")
    print(f"[rec] phase sequence: {' -> '.join(phases)}")
    print(f"[rec] max distance from home: {max(s['dist'] for s in samples):.0f} m")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--out", default="geofence.json")
    run(ap.parse_args())
