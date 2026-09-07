"""
Edge Case: Geofence Breach
---------------------------
Arm, take off, teleport GPS ~400m north (outside GEOFENCE_RADIUS_M=150m),
confirm forced RTL, climb, return to launch, settle, land.
"""
import time

# Establish world state -- sensor loops are already running and will
# deliver these values to the firmware on their next tick.
set_world(lat=36.123456, lon=-80.123456, fix=True, alt_ft=0.0, heading_deg=90.0)
send("FIX:1")
send("LAT:36.123456")
send("LON:-80.123456")
time.sleep(0.2)

send("CRSFSTART:1")
assert wait_for("[CRSF] START switch -- arming and taking off.", timeout=5), \
    "Drone did not arm"

for alt in [3, 6, 10, 13, 15]:
    set_world(alt_ft=float(alt))
    time.sleep(1.0)

assert wait_for_status(phase="HOLD", timeout=10), \
    "Never reached takeoff altitude"

# Teleport GPS ~400m north -- well outside geofence
set_world(lat=36.127056, alt_ft=15.0)
assert wait_for_status(phase="RTL", rtl="CLIMB", timeout=5), \
    "Geofence breach was not detected"

# RTL_CLIMB: confirm the firmware is actually commanding more thrust to
# climb, not just watching an injected barometer value cross a threshold.
# Query right after the breach (large alt error -> should demand real climb
# thrust) and compare against the hover baseline queried once cruise is
# reached -- comparing against a pre-breach HOLD-phase sample doesn't work
# because dashboard_rtl is only live once physicsTick_RTL is actually running.
climb_sample = wait_for_motor_base(0.05, timeout=3)
assert climb_sample is not None, "No [MOTOR] response during RTL_CLIMB"

for alt in [20, 30, 40, 50, 61]:
    set_world(alt_ft=float(alt))
    time.sleep(0.5)

assert wait_for_status(phase="RTL", rtl="RETURN", timeout=10), \
    "RTL climb never completed"

hover_sample = query_motor()
assert hover_sample is not None, "No [MOTOR] response after climb completed"
assert climb_sample["base"] > hover_sample["base"] + 0.05, \
    (f"Motor base throttle during climb ({climb_sample['base']}) was not "
     f"above cruise hover throttle ({hover_sample['base']}) -- firmware "
     "may not be commanding real climb thrust")

# RTL_RETURN: walk GPS back to launch and confirm the firmware is issuing a
# steering correction (non-zero roll/pitch mix) toward the launch heading,
# not just polling GPS until it happens to match the launch coordinates.
for lat in [36.125000, 36.123500, 36.123456]:
    set_world(lat=lat, alt_ft=61.0)
    assert wait_for_gps_publish(), \
        "GPS loop never published the updated position"
    sample = query_motor()
    assert sample is not None, "No [MOTOR] response while returning to launch"
    assert abs(sample["roll"]) > 0.01 or abs(sample["pitch"]) > 0.01, \
        "No steering correction commanded while returning to launch"

assert wait_for_status(phase=("HOVER_SETTLE", "LANDING", "LANDED"), timeout=10), \
    "Never arrived over launch pad"

# RTL_SETTLE: 3s hover. Keep barometer at 61 ft so transitionTo(PHASE_LANDING)
# snapshots the real cruise altitude, not zero.
set_world(alt_ft=61.0)
assert wait_for_status(phase="LANDING", timeout=8), \
    "Settle never completed"

# Landing from 61 ft at 1.5 ft/s -> ~41s
for alt in [55, 45, 35, 25, 15, 8, 3, 0]:
    set_world(alt_ft=float(alt))
    time.sleep(5.0)

assert wait_for_status(phase="LANDED", timeout=15), \
    "Never landed"

# ── Whole-run FORBID checks ──────────────────────────────────────────────
forbid("[NAV] Reached waypoint")   # mission was never started
forbid("[PANIC]")
