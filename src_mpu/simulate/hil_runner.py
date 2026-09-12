#!/usr/bin/env python3
"""
Hardware-in-the-Loop (HIL) test runner for Map The Farm drone firmware.

Connects to the real ESP32 over USB serial and runs continuous sensor loops
in background threads -- barometer at ~10 Hz, GPS at ~1 Hz, compass at ~10 Hz.
Scenario scripts mutate a shared world-state dict and the sensor threads do
the rest. No Wokwi, no YAML, no race conditions.

Usage:
    python3 hil_runner.py --port /dev/tty.usbmodem14101 --scenario scenarios/edge_geofence_breach.py

The firmware must be built with WOKWI_SIM defined:
    pio run -e wokwi_sim --target upload
"""

import argparse
import importlib.util
import sys
import threading
import time

import serial


# ---------------------------------------------------------------------------
# Shared world state
# ---------------------------------------------------------------------------
world = {
    "alt_ft":      0.0,
    "lat":         36.123456,
    "lon":         -80.123456,
    "fix":         False,
    "heading_deg": 90.0,
}
world_lock = threading.Lock()


# ---------------------------------------------------------------------------
# Serial connection
# ---------------------------------------------------------------------------
ser = None
serial_write_lock = threading.Lock()

def send(line: str):
    with serial_write_lock:
        ser.write((line + "\n").encode())


# ---------------------------------------------------------------------------
# Log capture
# ---------------------------------------------------------------------------
log_lines: list[str] = []
log_condition = threading.Condition()
status_request_lock = threading.Lock()
next_status_request_id = 0


def _complete_lines(rx_buffer: bytearray, raw: bytes):
    """Append raw serial bytes and return only complete newline-delimited lines."""
    rx_buffer.extend(raw)
    lines = []
    while b"\n" in rx_buffer:
        raw_line, _, remainder = rx_buffer.partition(b"\n")
        rx_buffer[:] = remainder
        line = raw_line.decode("utf-8", errors="replace").rstrip("\r")
        if line:
            lines.append(line)
    return lines


_PHASE_DESCRIPTIONS = {
    "PARKED": "on the ground and ready",
    "RAISE": "climbing to takeoff altitude",
    "HOLD": "hovering in place",
    "MISSION": "following the mission route",
    "RTL": "returning to the launch point",
    "HOVER_SETTLE": "hovering over the launch point before landing",
    "LANDING": "descending to land",
    "LANDED": "on the ground with motors disarmed",
}

_RTL_DESCRIPTIONS = {
    "CLIMB": "climbing to RTL altitude",
    "RETURN": "flying toward the launch point",
    "SETTLE": "settling over the launch point",
}


def _friendly_status(line: str):
    """Turn a machine-readable status response into a human-readable display."""
    match = _STATUS_RE.search(line)
    if not match:
        return None

    status = match.groupdict()
    phase = status["phase"]
    rtl = status["rtl"]
    gate = status["gate"]
    phase_description = _PHASE_DESCRIPTIONS.get(phase, "in an unknown phase")
    if phase == "RTL":
        phase_description += f"; RTL {_RTL_DESCRIPTIONS.get(rtl, 'has an unknown sub-state')}"

    if gate == "NONE":
        gate_description = "no approval pending"
    else:
        gate_description = f"waiting for approval to enter {gate}"

    return f"[STATE] Drone is {phase_description}. Gate: {gate_description}."

def _log_reader():
    rx_buffer = bytearray()
    last_display = None
    while True:
        try:
            # USB CDC is a byte stream. Keep partial data until its newline
            # arrives instead of treating a serial timeout as end-of-line.
            raw = ser.read(ser.in_waiting or 1)
            if not raw:
                continue
            for line in _complete_lines(rx_buffer, raw):
                with log_condition:
                    log_lines.append(line)
                    log_condition.notify_all()
                display = _friendly_status(line)
                if display is not None:
                    if display == last_display:
                        continue
                    last_display = display
                    print(display, flush=True)
                else:
                    last_display = None
                    print(f"[FW] {line!r}", flush=True)
        except Exception as e:
            print(f"[HIL] Log reader error: {e}", flush=True)
            break

def wait_for(substring: str, timeout: float = 60.0) -> bool:
    deadline = time.monotonic() + timeout
    with log_condition:
        next_line = len(log_lines)
    while time.monotonic() < deadline:
        with log_condition:
            while next_line == len(log_lines):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                log_condition.wait(timeout=remaining)
            new_lines = log_lines[next_line:]
            next_line = len(log_lines)
        if any(substring in line for line in new_lines):
            return True
    return False

def forbid(substring: str):
    with log_condition:
        hits = [l for l in log_lines if substring in l]
    assert not hits, f"FORBID violated: '{substring}' found in log: {hits[0]!r}"

def forbid_count_gt_1(substring: str):
    with log_condition:
        count = sum(1 for l in log_lines if substring in l)
    assert count <= 1, f"FORBID_COUNT_GT_1 violated: '{substring}' appeared {count} times"


# ---------------------------------------------------------------------------
# Motor telemetry -- request/response only (firmware never pushes this
# unsolicited), so it can't collide on the wire with safety/phase-transition
# log lines emitted from the nav task. Query it explicitly between waits.
# ---------------------------------------------------------------------------
import re as _re

_MOTOR_RE = _re.compile(
    r"\[MOTOR\] base=(?P<base>-?[\d.]+) roll=(?P<roll>-?[\d.]+) "
    r"pitch=(?P<pitch>-?[\d.]+)"
)
_STATUS_RE = _re.compile(
    r"\[STATUS\] id=(?P<id>\d+) phase=(?P<phase>[A-Z_]+) "
    r"rtl=(?P<rtl>[A-Z]+) gate=(?P<gate>[A-Z_]+) "
    r"reason=(?P<reason>[A-Z_]+) wp=(?P<wp>\d+)"
)

def query_motor(timeout: float = 3.0):
    """Send MOTOR? and block for the matching [MOTOR] response line.

    Returns a dict with keys base/roll/pitch, reflecting the mix the
    firmware actually computed for the ESCs, not just injected sensor
    state -- used to assert the firmware is really commanding the
    motors (climb throttle, steering correction) rather than only
    transitioning phases on scripted sensor values.
    """
    with log_condition:
        next_line = len(log_lines)
    send("MOTOR?")
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        with log_condition:
            while next_line == len(log_lines):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                log_condition.wait(timeout=remaining)
            new_lines = log_lines[next_line:]
            next_line = len(log_lines)
        for line in new_lines:
            m = _MOTOR_RE.search(line)
            if m:
                return {k: float(v) for k, v in m.groupdict().items()}
    return None

def wait_for_motor_base(minimum: float, timeout: float = 3.0):
    """Wait for physicsTask to publish a non-stale motor sample."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        sample = query_motor(timeout=min(0.5, deadline - time.monotonic()))
        if sample is not None and sample["base"] >= minimum:
            return sample
    return None

def query_status(timeout: float = 1.0):
    """Return a status snapshot correlated to this request, without side effects."""
    global next_status_request_id
    with status_request_lock:
        next_status_request_id += 1
        request_id = next_status_request_id
    with log_condition:
        next_line = len(log_lines)
    send(f"STATUS?{request_id}")
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        with log_condition:
            while next_line == len(log_lines):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                log_condition.wait(timeout=remaining)
            new_lines = log_lines[next_line:]
            next_line = len(log_lines)
        for line in new_lines:
            match = _STATUS_RE.search(line)
            if match and int(match.group("id")) == request_id:
                status = match.groupdict()
                status["wp"] = int(status["wp"])
                return status
    return None

def _matches(actual, expected):
    if expected is None:
        return True
    if isinstance(expected, (str, int, float, bool)):
        return actual == expected
    return actual in set(expected)

def wait_for_status(phase=None, rtl=None, gate=None, reason=None, wp=None, timeout=10.0):
    """Poll a durable firmware snapshot without modifying firmware state."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        status = query_status(timeout=min(1.0, remaining))
        if status and _matches(status["phase"], phase) and \
                      _matches(status["rtl"], rtl) and \
                      _matches(status["gate"], gate) and \
                      _matches(status["reason"], reason) and \
                      _matches(status["wp"], wp):
            return status
        time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
    return None

def approve_gate(phase: str, reason=None, timeout: float = 10.0):
    """Explicitly approve one expected transition and confirm it completed."""
    status = wait_for_status(gate=phase, reason=reason, timeout=timeout)
    if status is None:
        return None
    send(f"ALLOW:{phase}")
    print(f"[HIL] Approved phase transition to {phase}", flush=True)
    return wait_for_status(phase=phase, gate="NONE", timeout=timeout)


# ---------------------------------------------------------------------------
# Sensor loops
# ---------------------------------------------------------------------------

def _barometer_loop(interval_s: float = 0.1):
    while True:
        with world_lock:
            alt = world["alt_ft"]
        send(f"ALT:{alt:.2f}")
        time.sleep(interval_s)

gps_tick = 0
gps_tick_cond = threading.Condition()

def _gps_loop(interval_s: float = 1.0):
    global gps_tick
    while True:
        with world_lock:
            lat = world["lat"]
            lon = world["lon"]
            fix = world["fix"]
        send(f"FIX:{1 if fix else 0}")
        if fix:
            send(f"LAT:{lat:.6f}")
            send(f"LON:{lon:.6f}")
        with gps_tick_cond:
            gps_tick += 1
            gps_tick_cond.notify_all()
        time.sleep(interval_s)

def wait_for_gps_publish(n: int = 2, timeout: float = 5.0) -> bool:
    """Block until the GPS loop has published at least `n` more times.

    Scenarios call this after set_world(lat=..., lon=...) instead of
    sleeping a guessed duration. n=2 covers the race where set_world()
    lands between the loop's world read and its next publish -- the
    first tick may still send the old position, the second is
    guaranteed to see the update.
    """
    deadline = time.monotonic() + timeout
    with gps_tick_cond:
        target = gps_tick + n
        while gps_tick < target:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            gps_tick_cond.wait(remaining)
    return True

def _compass_loop(interval_s: float = 0.1):
    while True:
        with world_lock:
            hdg = world["heading_deg"]
        send(f"HDG:{hdg:.1f}")
        time.sleep(interval_s)


# ---------------------------------------------------------------------------
# Convenience helpers
# ---------------------------------------------------------------------------

def set_world(**kwargs):
    with world_lock:
        world.update(kwargs)

def arm_and_takeoff(takeoff_timeout: float = 15.0) -> bool:
    send("CRSFSTART:1")
    return approve_gate("RAISE", timeout=takeoff_timeout) is not None

def start_mission() -> bool:
    send("CRSFSTART:1")
    return wait_for("[CRSF] START switch -- starting waypoint mission.", timeout=5.0)

def emergency_stop():
    send("CRSFSTOP:1")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="HIL test runner for Map The Farm drone")
    parser.add_argument("--port",     required=True, help="Serial port, e.g. /dev/tty.usbmodem14101")
    parser.add_argument("--baud",     default=115200, type=int)
    parser.add_argument("--scenario", required=True, help="Path to scenario .py file")
    parser.add_argument("--timeout",  default=300,   type=int, help="Overall scenario timeout in seconds")
    args = parser.parse_args()

    global ser

    # Open the port once and keep it open -- same pattern that works in
    # pio device monitor. Opening triggers a board reset on ESP32-S3.
    # We start reading immediately and ping until the firmware responds.
    print(f"[HIL] Opening {args.port}...")
    ser = serial.Serial(args.port, args.baud, timeout=1.0)

    threading.Thread(target=_log_reader,     daemon=True).start()
    threading.Thread(target=_barometer_loop, daemon=True).start()
    threading.Thread(target=_gps_loop,       daemon=True).start()
    threading.Thread(target=_compass_loop,   daemon=True).start()

    # Ping until firmware responds -- covers the board reset + WiFi connect time
    print("[HIL] Waiting for firmware (PING handshake)...")
    ready = False
    for _ in range(40):
        send("PING:")
        if wait_for("[HIL] Ready.", timeout=0.5):
            ready = True
            break

    if not ready:
        print("[HIL] No PING response -- check port and that WOKWI_SIM firmware is flashed.")
        sys.exit(1)

    # Serial reconnects do not reliably reset every ESP32-S3 USB session.
    # Reset the simulation state explicitly so scenarios can run in a loop.
    # Right after an upload the firmware is still draining the boot-time
    # backlog of buffered sensor lines, so a single RESET can miss its ack --
    # retry it the same way the PING handshake above does.
    reset_ok = False
    for _ in range(10):
        send("RESET:")
        if wait_for("[HIL] State reset.", timeout=1.0):
            reset_ok = True
            break
    if not reset_ok:
        print("[HIL] State reset was not acknowledged.")
        sys.exit(1)

    print(f"[HIL] Firmware ready. Loading scenario: {args.scenario}")

    spec = importlib.util.spec_from_file_location("scenario", args.scenario)
    mod  = importlib.util.module_from_spec(spec)

    mod.world             = world
    mod.world_lock        = world_lock
    mod.log_lines         = log_lines
    mod.log_lock          = log_condition
    mod.set_world         = set_world
    mod.wait_for          = wait_for
    mod.wait_for_gps_publish = wait_for_gps_publish
    mod.forbid            = forbid
    mod.forbid_count_gt_1 = forbid_count_gt_1
    mod.query_motor       = query_motor
    mod.wait_for_motor_base = wait_for_motor_base
    mod.query_status      = query_status
    mod.wait_for_status   = wait_for_status
    mod.approve_gate      = approve_gate
    mod.send              = send
    mod.arm_and_takeoff   = arm_and_takeoff
    mod.start_mission     = start_mission
    mod.emergency_stop    = emergency_stop

    result = {"passed": False}

    def run_scenario():
        try:
            spec.loader.exec_module(mod)
            result["passed"] = True
        except AssertionError as e:
            print(f"\n[HIL] FAIL: {e}")
        except Exception as e:
            import traceback
            print(f"\n[HIL] Scenario error: {e}")
            traceback.print_exc()

    t = threading.Thread(target=run_scenario, daemon=True)
    t.start()
    t.join(timeout=args.timeout)

    if t.is_alive():
        print(f"\n[HIL] TIMEOUT after {args.timeout}s")
        sys.exit(1)

    if not result["passed"]:
        print("\nRESULT: FAIL")
        sys.exit(1)

    print("\nRESULT: PASS")
    sys.exit(0)


if __name__ == "__main__":
    main()
