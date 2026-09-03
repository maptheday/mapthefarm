#!/usr/bin/env python3
"""
Hardware-in-the-Loop (HIL) test runner for Map The Farm drone firmware.

Connects to the real ESP32 over USB serial and runs continuous sensor loops
in background threads — barometer at ~10 Hz, GPS at ~1 Hz, compass at ~10 Hz.
Scenario scripts mutate a shared world-state dict and the sensor threads read
from it on every tick.

Usage:
    python3 hil_runner.py --port /dev/tty.usbmodem14101 --scenario scenarios/full_flight_test.py
    python3 hil_runner.py --port /dev/tty.usbmodem14101 --scenario scenarios/edge_geofence_breach.py
    python3 hil_runner.py --port /dev/tty.usbmodem14101 --scenario scenarios/edge_gps_permanent_loss.py
    python3 hil_runner.py --port /dev/tty.usbmodem14101 --scenario scenarios/edge_max_flight_timeout.py

The firmware must be built with WOKWI_SIM defined:
    pio run -e wokwi_sim --target upload
"""

import argparse
import importlib.util
import queue
import sys
import threading
import time

import serial


# ---------------------------------------------------------------------------
# Shared world state
# Every sensor thread reads from this dict on each tick. Scenario scripts
# write to it to simulate moving the drone, losing GPS, etc.
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
# Serial connection (shared by all threads; writes are serialised by a lock)
# ---------------------------------------------------------------------------
ser = None
serial_write_lock = threading.Lock()

def send(line: str):
    """Write one newline-terminated command to the firmware."""
    with serial_write_lock:
        ser.write((line + "\n").encode())


# ---------------------------------------------------------------------------
# Log capture thread
# ---------------------------------------------------------------------------
log_queue: queue.Queue = queue.Queue()
log_lines: list = []
log_lock  = threading.Lock()

def _log_reader():
    while True:
        try:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").rstrip()
            if not line:
                continue
            with log_lock:
                log_lines.append(line)
            log_queue.put(line)
            print(f"[FW] {line!r}", flush=True)
        except Exception as e:
            print(f"[HIL] Log reader error: {e}", flush=True)
            break

def wait_for(substring: str, timeout: float = 60.0) -> bool:
    """Block until a firmware log line containing `substring` appears."""
    print(f"[HIL] wait_for: {substring!r}", flush=True)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        try:
            line = log_queue.get(timeout=min(remaining, 0.5))
            print(f"[HIL] got line: {line!r}", flush=True)
            if substring in line:
                return True
        except queue.Empty:
            pass
    print(f"[HIL] wait_for TIMED OUT: {substring!r}", flush=True)
    return False

def forbid(substring: str):
    """
    Assert that `substring` never appeared anywhere in the log.
    Call this at the end of a scenario for whole-run FORBID checks.
    """
    with log_lock:
        hits = [l for l in log_lines if substring in l]
    assert not hits, f"FORBID violated: '{substring}' found in log: {hits[0]!r}"

def forbid_count_gt_1(substring: str):
    """Assert that `substring` appeared at most once in the full log."""
    with log_lock:
        count = sum(1 for l in log_lines if substring in l)
    assert count <= 1, f"FORBID_COUNT_GT_1 violated: '{substring}' appeared {count} times"


# ---------------------------------------------------------------------------
# Sensor loops
# ---------------------------------------------------------------------------

def _barometer_loop(interval_s: float = 0.1):
    while True:
        with world_lock:
            alt = world["alt_ft"]
        send(f"ALT:{alt:.2f}")
        time.sleep(interval_s)

def _gps_loop(interval_s: float = 1.0):
    while True:
        with world_lock:
            lat = world["lat"]
            lon = world["lon"]
            fix = world["fix"]
        send(f"FIX:{1 if fix else 0}")
        if fix:
            send(f"LAT:{lat:.6f}")
            send(f"LON:{lon:.6f}")
        time.sleep(interval_s)

def _compass_loop(interval_s: float = 0.1):
    while True:
        with world_lock:
            hdg = world["heading_deg"]
        send(f"HDG:{hdg:.1f}")
        time.sleep(interval_s)


# ---------------------------------------------------------------------------
# Convenience helpers for scenario scripts
# ---------------------------------------------------------------------------

def set_world(**kwargs):
    """Update one or more world-state values atomically."""
    with world_lock:
        world.update(kwargs)

def arm_and_takeoff(takeoff_timeout: float = 15.0) -> bool:
    send("CRSFSTART:1")
    return wait_for("[NAV] Takeoff altitude reached, transitioning to HOLD.", timeout=takeoff_timeout)

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

    # ESP32-S3 native USB CDC re-enumeration:
    # Opening the port resets the board. The board drops off USB,
    # reboots, then comes back on the same port name. We poll until
    # the port is actually readable again, then start the log reader.
    print(f"[HIL] Opening {args.port} to trigger reset...")
    try:
        trigger = serial.Serial(args.port, args.baud, timeout=0.1)
        time.sleep(0.5)
        trigger.close()
    except Exception:
        pass

    # Poll until the port comes back
    print("[HIL] Waiting for board to re-enumerate...")
    ser = None
    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline:
        try:
            ser = serial.Serial(args.port, args.baud, timeout=1.0)
            print(f"[HIL] Port back up. Starting log reader.")
            break
        except Exception:
            time.sleep(0.25)

    if ser is None:
        print("[HIL] Board never came back on port — is it plugged in?")
        sys.exit(1)

    # Log reader starts the moment the port is readable
    threading.Thread(target=_log_reader, daemon=True).start()

    # 2. Perform the PING/READY handshake to guarantee firmware is ready
    print("[HIL] Handshaking with firmware (sending PING:)...")
    ping_success = False
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        send("PING:")
        if wait_for("READY", timeout=1.0):
            ping_success = True
            print("[HIL] Board responded with READY! Handshake complete.")
            break
        time.sleep(0.5)

    if not ping_success:
        print("[HIL] ERROR: Board failed to respond to PING: within 10 seconds.")
        sys.exit(1)
        
    threading.Thread(target=_barometer_loop, daemon=True).start()
    threading.Thread(target=_gps_loop,       daemon=True).start()
    threading.Thread(target=_compass_loop,   daemon=True).start()

    print(f"[HIL] Loading scenario: {args.scenario}")

    spec = importlib.util.spec_from_file_location("scenario", args.scenario)
    mod  = importlib.util.module_from_spec(spec)

    mod.world          = world
    mod.world_lock     = world_lock
    mod.log_lines      = log_lines
    mod.log_lock       = log_lock
    mod.set_world      = set_world
    mod.wait_for       = wait_for
    mod.forbid         = forbid
    mod.forbid_count_gt_1 = forbid_count_gt_1
    mod.send           = send
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