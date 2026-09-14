"""
Manual Flight Test
------------------
Exercises MANUAL (RC-stick) mode end to end:
  - enter MANUAL from PARKED via the MANUAL switch
  - throttle stick spins the motors up / holds altitude when centered
  - roll/pitch sticks command lean angle
  - position hold: centered sticks drop a GPS anchor and lean back on drift
  - a moved stick overrides position hold (and drops the anchor)
  - MANUAL switch off hands back to auto-hover (HOLD)
  - STOP still cuts to PARKED from MANUAL

Run with MANUAL_POSITION_HOLD = true (the firmware default). GPS fix is kept
on throughout so position hold has something to hold to.
"""
import time

# ── Establish a GPS fix and a known starting position on the ground ──────
set_world(lat=36.123456, lon=-80.123456, fix=True, alt_ft=0.0, heading_deg=90.0)
assert wait_for_gps_publish(), "GPS loop never published the initial fix"

# ── Enter MANUAL from PARKED (sticks centered) ───────────────────────────
set_sticks(throttle=0.5, roll=0.0, pitch=0.0, yaw=0.0)
send("CRSFMANUAL:1")
assert approve_gate("MANUAL", reason="MANUAL_ON", timeout=5), "MANUAL switch did not take control"

# On the ground with the throttle centered, the target altitude sits at 0 and
# the motors stay near idle (no built-in hover throttle -- base comes from the
# altitude error, which is ~0 here).
m = query_manual()
assert m is not None, "No [MANUAL] telemetry"
assert m["targetAlt"] < 0.5, f"Expected ground-level target altitude, got {m['targetAlt']}"
assert m["base"] < 0.10, f"Expected idle motors with throttle centered, got base={m['base']}"

# ── Throttle up: target altitude climbs and the motors spin up ───────────
set_sticks(throttle=1.0)     # full up
time.sleep(1.5)              # ~3 ft/s -> ~4.5 ft of target climb
m = query_manual()
assert m["targetAlt"] > 2.0, f"Throttle up should raise target altitude, got {m['targetAlt']}"
assert m["base"] > 0.10, f"Throttle up should spin the motors, got base={m['base']}"

# ── Center throttle: altitude holds (target stops moving) ────────────────
set_sticks(throttle=0.5)
a = query_manual()["targetAlt"]
time.sleep(0.8)
b = query_manual()["targetAlt"]
assert abs(b - a) < 0.3, f"Centered throttle should hold altitude, drifted {a} -> {b}"

# ── Position hold: centered sticks drop an anchor, drift is corrected ─────
time.sleep(0.5)
m = query_manual()
assert m["anchored"] == 1, "Centered sticks with a GPS fix should drop an anchor"

# Drift ~16 m north of the anchor; the drone should lean back to cancel it.
set_world(lat=36.123600)
assert wait_for_gps_publish(), "GPS loop never published the drifted position"
time.sleep(0.3)
m = query_manual()
assert abs(m["targetPitch"]) > 2.0, \
    f"Position hold should lean back on drift, got targetPitch={m['targetPitch']}"

# ── A moved stick overrides position hold and drops the anchor ───────────
set_sticks(roll=1.0)         # full right roll
time.sleep(0.4)
m = query_manual()
assert m["targetRoll"] > 10.0, f"Full roll stick should command a big lean, got {m['targetRoll']}"
assert m["anchored"] == 0, "Moving a stick should release the position-hold anchor"

# Re-center -> it re-anchors at the new spot.
set_sticks(roll=0.0)
time.sleep(0.5)
assert query_manual()["anchored"] == 1, "Centering the sticks again should re-anchor"

# ── MANUAL switch off -> hand back to auto-hover (HOLD) ───────────────────
send("CRSFMANUAL:0")
assert approve_gate("HOLD", reason="MANUAL_OFF", timeout=5), "MANUAL off did not hand back to HOLD"

# ── STOP from HOLD still cuts to PARKED ──────────────────────────────────
send("CRSFSTOP:1")
assert approve_gate("PARKED", reason="EMERGENCY_STOP", timeout=5), "STOP did not return to PARKED"

# ── Whole-run FORBID checks ──────────────────────────────────────────────
forbid("[PANIC]")
