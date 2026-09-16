#!/usr/bin/env python3
"""
Record a REAL closed-loop "fly out -> geofence -> RTL home" flight to JSON,
flown by the firmware against RotorPy physics.

Home is placed 300 m due south of the first mission waypoint, so starting the
mission sends the drone north; it trips the 150 m geofence and the firmware
autonomously flips to RTL and flies home. Every tick we log where the drone
really is (per the firmware's own GPS) and which phase it's in.

Calibrated nav mapping (see scratchpad/calib_nav.py):
  * attitude:  firmware_roll = +rotorpy_roll,  firmware_pitch = -rotorpy_pitch
  * position:  north = -x[0],  east = -x[1]   (both flipped -> negative feedback)
  * yaw is pinned forward in the sim and we feed a heading matching the leg's
    target bearing, so the firmware's yaw controller stays quiet and its
    world-frame nav corrections map cleanly onto the fixed body frame.

Run:
  simulate/.venv-rotorpy/bin/python simulate/scenarios_rotor_py/record_rtl_flight.py \
      --port /dev/cu.usbmodem14101 --vehicle hummingbird --out flight.json
"""
import argparse, json, math, re, threading, time
import numpy as np
from scipy.spatial.transform import Rotation

from rotorpy_bridge import (Firmware, euler_deg, VEHICLES, FW_TO_RP,
                            ROLL_SIGN, PITCH_SIGN, FT_PER_M)
from rotorpy.vehicles.multirotor import Multirotor

# First mission waypoint; put home 300 m due south so the mission heads north.
WP0_LAT, WP0_LON = 36.123456, -80.123456
M_PER_DEG_LAT = 111320.0
HOME_LAT = WP0_LAT - 300.0 / M_PER_DEG_LAT
HOME_LON = WP0_LON
M_PER_DEG_LON = 111320.0 * math.cos(math.radians(HOME_LAT))
GEOFENCE_M = 150.0
_STATUS_RE = re.compile(r"\[STATUS\] id=\d+ phase=(?P<phase>[A-Z_]+) ")


def latlon(north_m, east_m):
    return HOME_LAT + north_m / M_PER_DEG_LAT, HOME_LON + east_m / M_PER_DEG_LON


class AutoGate(threading.Thread):
    """Approve HIL gates, RESENDING ALLOW until the firmware confirms it. USB-CDC
    can drop a line when we're streaming sensors fast, so one ALLOW isn't enough;
    we keep sending for the pending phase until we see its 'approved=' line."""
    def __init__(self, fw):
        super().__init__(daemon=True); self.fw = fw; self.seen = 0; self.stop = False
        self.pending = None
    def run(self):
        rq = re.compile(r"request=([A-Z_]+)")
        ok = re.compile(r"approved=([A-Z_]+)")
        while not self.stop:
            with self.fw.lock:
                new = self.fw.lines[self.seen:]; self.seen = len(self.fw.lines)
            for ln in new:
                m = rq.search(ln)
                if m: self.pending = m.group(1)
                m = ok.search(ln)
                if m and m.group(1) == self.pending: self.pending = None
            if self.pending:
                self.fw.send(f"ALLOW:{self.pending}")
            time.sleep(0.05)


def query_phase(fw, timeout=0.4):
    with fw.lock: start = len(fw.lines)
    fw.send("STATUS?1")
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        with fw.lock:
            for ln in fw.lines[start:]:
                m = _STATUS_RE.search(ln)
                if m: return m.group("phase")
            start = len(fw.lines)
        time.sleep(0.005)
    return None


def run(args):
    quad = VEHICLES[args.vehicle]
    veh = Multirotor(quad, control_abstraction="cmd_motor_speeds", aero=True, enable_ground=True)
    K_DRAG = args.drag   # optional extra horizontal air-drag (1/s); 0 = rely on aero only
    smax = quad["rotor_speed_max"]
    hover = math.sqrt(quad["mass"] * 9.81 / (4 * quad["k_eta"]))

    fw = Firmware(args.port); gate = AutoGate(fw); gate.start()
    print(f"[rec] connected {args.port}, vehicle={args.vehicle}")
    fw.send("RESET:"); fw.wait_for("State reset", 5)

    state = {"x": np.array([0.0, 0.0, 0.0]), "v": np.zeros(3),
             "q": Rotation.from_euler("XYZ", [0, 0, 0], degrees=True).as_quat(),
             "w": np.zeros(3), "wind": np.zeros(3), "rotor_speeds": np.full(4, hover)}
    fw.send("FIX:1")
    la, lo = latlon(0, 0)
    fw.send(f"LAT:{la:.6f}"); fw.send(f"LON:{lo:.6f}"); fw.send("ALT:0.0"); fw.send("HDG:0.0")
    time.sleep(0.5)

    samples, phase = [], "PARKED"
    armed = mission_started = False
    t0 = last = time.monotonic()

    while True:
        now = time.monotonic(); t = now - t0
        if t > args.seconds: print("[rec] time limit"); break
        dt = now - last; last = now
        if dt <= 0 or dt > 0.2: dt = 0.02

        if not armed and t > 1.0:
            fw.send("CRSFSTART:1"); armed = True; print("[rec] START (arm/takeoff)")
        if phase == "HOLD" and not mission_started:
            fw.send("CRSFSTART:1"); mission_started = True; print("[rec] START (mission -> north)")

        mix = fw.query_motor()
        if mix is None: continue
        cmd = np.array([mix[k] for k in FW_TO_RP]) * smax
        state = veh.step(state, {"cmd_motor_speeds": cmd}, dt)

        # Mild horizontal air drag: v_xy *= e^(-k*dt). Damps the RTL overshoot
        # (nav PIDs are aggressive over long distances) without touching climb.
        state["v"][0] *= math.exp(-K_DRAG * dt)
        state["v"][1] *= math.exp(-K_DRAG * dt)

        # Pin yaw forward (isolate translation); keep the calibrated attitude read.
        r, p, _ = euler_deg(state["q"])
        state["q"] = Rotation.from_euler("XYZ", [r, p, 0], degrees=True).as_quat()
        state["w"][2] = 0.0

        north = -state["x"][0]          # calibrated position mapping
        east = -state["x"][1]
        up_m = state["x"][2]
        # Feed a heading matching the leg's bearing so the yaw controller stays quiet:
        # outbound/climb target is north (0 deg); once returning, home is south (180).
        hdg = 180.0 if phase in ("RTL_RETURN", "RTL_SETTLE", "LANDING", "LANDED") else 0.0

        # Keep serial traffic light so the gate's ALLOW lines are never starved:
        # IMU + ALT drive the fast control loop every tick; GPS/heading move slowly
        # in the real world so send them at ~half rate; poll phase less often still.
        n = len(samples)
        la, lo = latlon(north, east)
        fw.send(f"IMU:{ROLL_SIGN*r:.3f},{PITCH_SIGN*p:.3f},0.000")
        fw.send(f"ALT:{up_m*FT_PER_M:.2f}")
        if n % 2 == 0:
            fw.send(f"HDG:{hdg:.1f}")
            fw.send(f"LAT:{la:.6f}"); fw.send(f"LON:{lo:.6f}")

        if n % 4 == 0:
            pnew = query_phase(fw)
            if pnew: phase = pnew

        dist = math.hypot(north, east)
        samples.append({"t": round(t, 2), "north": round(north, 1), "east": round(east, 1),
                        "up": round(up_m, 1), "phase": phase, "dist": round(dist, 1),
                        "heading": hdg})
        if len(samples) % 20 == 0:
            print(f"[rec] t={t:5.1f}s {phase:12s} N={north:6.1f} E={east:6.1f} up={up_m:5.1f} "
                  f"dist={dist:6.1f} base={mix['base']:.2f} roll={r:+5.1f} pitch={p:+5.1f}")
        if phase == "LANDED" and t > 5:
            print("[rec] landed -- done"); break

        # Pace the loop (~25 Hz) so we don't overflow the ESP's USB-serial buffer.
        time.sleep(max(0.0, 0.04 - (time.monotonic() - now)))

    gate.stop = True; fw.close()
    out = {"home": {"lat": HOME_LAT, "lon": HOME_LON}, "geofence_m": GEOFENCE_M,
           "vehicle": args.vehicle, "samples": samples}
    with open(args.out, "w") as f: json.dump(out, f)
    seq = []
    for s in samples:
        if not seq or seq[-1] != s["phase"]: seq.append(s["phase"])
    print(f"\n[rec] wrote {len(samples)} samples -> {args.out}")
    print(f"[rec] phases: {' -> '.join(seq)}")
    print(f"[rec] max distance from home: {max((s['dist'] for s in samples), default=0):.0f} m")
    print(f"[rec] final distance from home: {samples[-1]['dist'] if samples else 0:.0f} m")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--vehicle", default="hummingbird", choices=list(VEHICLES))
    ap.add_argument("--seconds", type=float, default=120.0)
    ap.add_argument("--out", default="flight.json")
    ap.add_argument("--drag", type=float, default=0.0, help="extra horizontal drag (1/s)")
    run(ap.parse_args())
