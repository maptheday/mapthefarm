#!/usr/bin/env python3
"""
RotorPy <-> firmware CLOSED-LOOP bridge (experimental)
======================================================
Runs the REAL firmware (WOKWI_SIM build, on the ESP) against a RotorPy physics
model, closing the loop the plain HIL runner can't: the physics reacts to the
motors, so we can watch the attitude controller actually settle (or diverge)
instead of only checking it pushes the right way.

Every tick:
  1. ask the firmware for its motor mix          (MOTOR? -> m1..m4, throttle 0..1)
  2. turn those throttles into rotor speeds and step RotorPy by the real dt
  3. read RotorPy's new attitude/altitude and feed it back  (IMU: / ALT: / HDG:)

We fly MANUAL with level, centred sticks, then apply a one-time ATTITUDE KICK
(a sudden tilt, like a gust) and watch whether the firmware brings it back.

IMPORTANT -- what this proves and doesn't:
  * It closes the loop against a *model* (default: the RotorPy crazyflie). It can
    show correct-sign recovery and gross (in)stability.
  * It CANNOT validate real tuning: the firmware's PID gains were written for a
    different airframe than this model, and the loop runs at serial speed, not
    200 Hz. Divergence here can mean tuning/loop-rate mismatch, not a code bug.
  * Nothing replaces a tethered bench test.

Usage:
  simulate/.venv-rotorpy/bin/python simulate/rotorpy_bridge.py --port /dev/cu.usbmodem14101
"""
import argparse
import math
import re
import threading
import time

import numpy as np
import serial
from scipy.spatial.transform import Rotation

from rotorpy.vehicles.multirotor import Multirotor
from rotorpy.vehicles.crazyflie_params import quad_params as CRAZYFLIE
from rotorpy.vehicles.hummingbird_params import quad_params as HUMMINGBIRD

VEHICLES = {"crazyflie": CRAZYFLIE, "hummingbird": HUMMINGBIRD}

FT_PER_M = 3.28084
RAD2DEG = 180.0 / math.pi

# Firmware motor order is M1=FL, M2=FR, M3=RL, M4=RR (EspESC.hpp).
# RotorPy rotor order is r1=FL, r2=FR, r3=RR, r4=RL.
# So the RotorPy command array [r1,r2,r3,r4] = [M1, M2, M4, M3].
FW_TO_RP = ["m1", "m2", "m4", "m3"]

# Sign mapping firmware<-rotorpy, calibrated empirically (see rp_calib):
#   firmware_roll  = +rotorpy_roll     (both: banked right = positive)
#   firmware_pitch = -rotorpy_pitch    (rotorpy nose-up = negative; firmware = positive)
ROLL_SIGN = +1.0
PITCH_SIGN = -1.0

_MOTOR_RE = re.compile(
    r"\[MOTOR\] base=(?P<base>-?[\d.]+) roll=(?P<roll>-?[\d.]+) "
    r"pitch=(?P<pitch>-?[\d.]+) m1=(?P<m1>-?[\d.]+) m2=(?P<m2>-?[\d.]+) "
    r"m3=(?P<m3>-?[\d.]+) m4=(?P<m4>-?[\d.]+)"
)


class Firmware:
    """Thin serial link to the WOKWI_SIM firmware with a background reader."""

    def __init__(self, port, baud=115200):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.lines = []
        self.lock = threading.Lock()
        self._wlock = threading.Lock()   # serialize writes across threads
        self._stop = False
        self._buf = bytearray()
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        while not self._stop:
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
            except Exception:
                break
            if not data:
                continue
            self._buf.extend(data)
            while b"\n" in self._buf:
                raw, _, self._buf = self._buf.partition(b"\n")
                line = raw.decode(errors="replace").strip("\r")
                with self.lock:
                    self.lines.append(line)

    def send(self, line):
        with self._wlock:
            self.ser.write((line + "\n").encode())

    def wait_for(self, substr, timeout=5.0):
        deadline = time.monotonic() + timeout
        seen = 0
        while time.monotonic() < deadline:
            with self.lock:
                for ln in self.lines[seen:]:
                    if substr in ln:
                        return ln
                seen = len(self.lines)
            time.sleep(0.005)
        return None

    def query_motor(self, timeout=1.0):
        with self.lock:
            start = len(self.lines)
        self.send("MOTOR?")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                for ln in self.lines[start:]:
                    m = _MOTOR_RE.search(ln)
                    if m:
                        return {k: float(v) for k, v in m.groupdict().items()}
                start = len(self.lines)
            time.sleep(0.002)
        return None

    def approve_gate(self, phase, timeout=5.0):
        # Firmware announces "[HIL_GATE] request=<phase> ..." then waits for ALLOW.
        if self.wait_for(f"request={phase}", timeout=timeout) is None:
            return False
        self.send(f"ALLOW:{phase}")
        return self.wait_for(f"approved={phase}", timeout=timeout) is not None

    def close(self):
        self._stop = True
        time.sleep(0.1)
        self.ser.close()


def euler_deg(q):
    """q=[i,j,k,w] -> (roll, pitch, yaw) degrees, XYZ intrinsic (RotorPy's convention)."""
    return Rotation.from_quat(q).as_euler("XYZ", degrees=True)


def run(args):
    quad = VEHICLES[args.vehicle]
    veh = Multirotor(quad, control_abstraction="cmd_motor_speeds",
                     aero=False, enable_ground=False)
    smax = quad["rotor_speed_max"]

    fw = Firmware(args.port)
    print(f"[bridge] connected {args.port}, vehicle={args.vehicle}, rotor_speed_max={smax}")

    # Fresh firmware state, then enter MANUAL from the ground (no GPS fix -> MANUAL
    # position-hold is off, so centred sticks give level targets).
    fw.send("RESET:")
    fw.wait_for("State reset", timeout=5)
    # Start already airborne at a hover so we test ATTITUDE, not takeoff.
    start_alt_m = 1.0
    fw.send(f"ALT:{start_alt_m * FT_PER_M:.2f}")
    fw.send("HDG:0.0")
    fw.send("STICKS:0.5,0,0,0")     # throttle centred (hold), sticks level
    time.sleep(0.3)
    fw.send("CRSFMANUAL:1")
    if not fw.approve_gate("MANUAL"):
        print("[bridge] ERROR: could not enter MANUAL")
        fw.close()
        return
    print("[bridge] in MANUAL, hovering")

    # RotorPy state: airborne, level, rotors at hover.
    hover = math.sqrt(quad["mass"] * 9.81 / (4 * quad["k_eta"]))
    state = {"x": np.array([0.0, 0.0, start_alt_m]), "v": np.zeros(3),
             "q": np.array([0, 0, 0, 1.0]), "w": np.zeros(3),
             "wind": np.zeros(3), "rotor_speeds": np.full(4, hover)}

    # Keep the firmware's altitude loop happy: nudge the throttle stick so its
    # target altitude sits near where we start (its PID has no hover feedforward).
    fw.send("STICKS:0.55,0,0,0")

    hist = {"t": [], "roll": [], "pitch": [], "alt": []}
    t0 = time.monotonic()
    last = t0
    kicked = False
    KICK_AT = args.kick_at
    RUN_FOR = args.seconds

    while True:
        now = time.monotonic()
        t = now - t0
        if t >= RUN_FOR:
            break
        dt = now - last
        last = now
        if dt <= 0 or dt > 0.2:
            dt = 0.02  # clamp weird gaps

        mix = fw.query_motor()
        if mix is None:
            print("[bridge] no MOTOR? response; retrying")
            continue

        # Throttles (0..1) -> rotor speeds (rad/s), reordered to RotorPy's rotors.
        cmd = np.array([mix[k] for k in FW_TO_RP]) * smax
        state = veh.step(state, {"cmd_motor_speeds": cmd}, dt)

        # One-time attitude KICK: snap in a roll disturbance, like a gust.
        if not kicked and t >= KICK_AT:
            r, p, y = euler_deg(state["q"])
            state["q"] = Rotation.from_euler(
                "XYZ", [args.kick_deg, 0.0, y], degrees=True).as_quat()
            kicked = True
            print(f"[bridge] t={t:5.2f}s  KICK: roll -> {args.kick_deg:+.0f} deg")

        roll, pitch, yaw = euler_deg(state["q"])
        alt_ft = state["x"][2] * FT_PER_M
        yaw_rate_dps = state["w"][2] * RAD2DEG

        # Feed the firmware what its sensors would report (with the calibrated signs).
        fw.send(f"IMU:{ROLL_SIGN*roll:.3f},{PITCH_SIGN*pitch:.3f},{yaw_rate_dps:.3f}")
        fw.send(f"ALT:{alt_ft:.2f}")
        fw.send(f"HDG:{(yaw + 360) % 360:.1f}")

        hist["t"].append(t)
        hist["roll"].append(roll)
        hist["pitch"].append(pitch)
        hist["alt"].append(state["x"][2])

        if len(hist["t"]) % 20 == 0:
            print(f"[bridge] t={t:5.2f}s roll={roll:+6.1f} pitch={pitch:+6.1f} "
                  f"alt={state['x'][2]:+5.2f}m base={mix['base']:.2f}")

    fw.send("STICKS:0.5,0,0,0")
    fw.close()
    verdict(hist, args)


def verdict(hist, args):
    if not hist["t"]:
        print("[bridge] no data collected.")
        return
    after = [(t, r) for t, r in zip(hist["t"], hist["roll"]) if t >= args.kick_at]
    roll_after = [abs(r) for _, r in after]
    peak = max(roll_after) if roll_after else 0.0
    settle = abs(hist["roll"][-1])
    print("\n==================== RESULT ====================")
    print(f"vehicle={args.vehicle}  kick=+{args.kick_deg:.0f} deg roll at t={args.kick_at}s")
    print(f"peak |roll| after kick : {peak:6.1f} deg")
    print(f"final |roll|           : {settle:6.1f} deg")
    if settle < 5.0 and peak < 60.0:
        print("VERDICT: RECOVERED -> the closed loop drove the tilt back to level.")
        print("         (correct-sign stabilization against this model)")
    elif settle < peak * 0.5:
        print("VERDICT: DAMPING but not fully settled in the window (loose tuning / slow loop).")
    else:
        print("VERDICT: DID NOT RECOVER -> diverged or oscillated.")
        print("         Could be a sign/mapping error OR tuning/loop-rate mismatch vs this model.")
    print("================================================")

    if args.plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
            a1.plot(hist["t"], hist["roll"], label="roll")
            a1.plot(hist["t"], hist["pitch"], label="pitch")
            a1.axvline(args.kick_at, color="r", ls="--", lw=1, label="kick")
            a1.axhline(0, color="k", lw=0.5)
            a1.set_ylabel("angle (deg)"); a1.legend(); a1.grid(alpha=0.3)
            a2.plot(hist["t"], hist["alt"], color="g", label="altitude")
            a2.axvline(args.kick_at, color="r", ls="--", lw=1)
            a2.set_ylabel("altitude (m)"); a2.set_xlabel("time (s)")
            a2.legend(); a2.grid(alpha=0.3)
            fig.suptitle(f"Closed-loop attitude recovery ({args.vehicle})")
            fig.tight_layout()
            fig.savefig(args.plot, dpi=110)
            print(f"[bridge] saved plot -> {args.plot}")
        except Exception as e:
            print(f"[bridge] plot failed: {e}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--vehicle", default="crazyflie", choices=list(VEHICLES))
    ap.add_argument("--seconds", type=float, default=12.0)
    ap.add_argument("--kick-at", type=float, default=3.0)
    ap.add_argument("--kick-deg", type=float, default=15.0)
    ap.add_argument("--plot", default=None, help="path to save a PNG plot")
    run(ap.parse_args())
