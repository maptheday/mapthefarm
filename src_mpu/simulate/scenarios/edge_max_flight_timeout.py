"""
Edge Case: Max Flight Time Timeout
------------------------------------
Arm, take off, sit in HOLD until MAX_FLIGHT_TIME_MS fires (60s in
WOKWI_SIM build), then RTL, climb, return, settle, land.

Only valid against firmware built with WOKWI_SIM defined.
"""
import time


set_world(lat=36.123456, lon=-80.123456, fix=True, alt_ft=0.0, heading_deg=90.0)
assert wait_for_gps_publish(), "GPS loop never published the initial fix"

send("CRSFSTART:1")
assert approve_gate("RAISE", reason="OPERATOR_START", timeout=5), "Drone did not arm"

# Barometer rising during RAISE (~5s)
for alt in [3, 6, 10, 13, 15]:
    set_world(alt_ft=float(alt))
    time.sleep(1.0)

assert approve_gate("HOLD", reason="TAKEOFF_COMPLETE", timeout=10), \
    "Never reached takeoff altitude"

# Sit in HOLD. armedAtMs was set at RAISE entry (~5s ago), so we need
# ~55s more for MAX_FLIGHT_TIME_MS (60s total) to fire. RTL is now three
# ordinary phases (RTL_CLIMB -> RTL_RETURN -> RTL_SETTLE), each a gated
# transition we approve in turn.
set_world(alt_ft=15.0)
assert approve_gate("RTL_CLIMB", reason="MAX_FLIGHT_TIME", timeout=60), \
    "Max flight time safety did not request RTL_CLIMB"

# Nudge GPS ~60m north (well inside the 150m geofence) so RTL_RETURN has real
# distance to fly back -- otherwise it would arrive on its first nav tick.
set_world(lat=36.124000, alt_ft=15.0)

# RTL_CLIMB: ramp barometer to RTL_ALTITUDE_FT (60 ft)
for alt in [20, 30, 40, 50, 61]:
    set_world(alt_ft=float(alt))
    time.sleep(0.5)

assert approve_gate("RTL_RETURN", reason="RTL_CLIMB_COMPLETE", timeout=10), \
    "RTL did not complete its climb transition"

# Walk GPS back to launch -> RTL_RETURN arrives -> RTL_SETTLE.
set_world(lat=36.123456, alt_ft=61.0)
assert wait_for_gps_publish(), "GPS loop never published the launch position"
assert approve_gate("RTL_SETTLE", reason="RTL_ARRIVED", timeout=10), \
    "RTL_RETURN never arrived over launch"

# RTL_SETTLE: 3s hover -> LANDING.
set_world(alt_ft=61.0)
assert approve_gate("LANDING", reason="RTL_COMPLETE", timeout=8), \
    "RTL settle did not enter landing"

# Landing from 61 ft at 1.5 ft/s -> ~41s
for alt in [55, 45, 35, 25, 15, 8, 3, 0]:
    set_world(alt_ft=float(alt))
    time.sleep(5.0)

assert approve_gate("LANDED", reason="TOUCHDOWN", timeout=15), \
    "Never landed"

# ── Whole-run FORBID checks ──────────────────────────────────────────────
forbid("[SAFETY] Geofence exceeded")   # timeout should fire before geofence
forbid("[PANIC]")
