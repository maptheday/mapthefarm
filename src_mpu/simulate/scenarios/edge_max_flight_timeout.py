"""
Edge Case: Max Flight Time Timeout
------------------------------------
Arm, take off, sit in HOLD until MAX_FLIGHT_TIME_MS fires (20s in
WOKWI_SIM build), then RTL, climb, return, settle, land.

Only valid against firmware built with WOKWI_SIM defined.
"""
import time


set_world(lat=36.123456, lon=-80.123456, fix=True, alt_ft=0.0, heading_deg=90.0)
assert wait_for_gps_publish(), "GPS loop never published the initial fix"

send("CRSFSTART:1")
assert wait_for("[CRSF] START switch -- arming and taking off.", timeout=5), \
    "Drone did not arm"

# Barometer rising during RAISE (~5s)
for alt in [3, 6, 10, 13, 15]:
    set_world(alt_ft=float(alt))
    time.sleep(1.0)

assert wait_for("[NAV] Takeoff altitude reached, transitioning to HOLD.", timeout=10), \
    "Never reached takeoff altitude"

# Sit in HOLD. armedAtMs was set at RAISE entry (~5s ago), so we need
# ~15s more for MAX_FLIGHT_TIME_MS (20s total) to fire.
set_world(alt_ft=15.0)
assert wait_for("[SAFETY] Max flight time reached — forcing RTL.", timeout=20), \
    "Max flight time safety never triggered"

# RTL_CLIMB: ramp barometer to RTL_ALTITUDE_FT (60 ft)
for alt in [20, 30, 40, 50, 61]:
    set_world(alt_ft=float(alt))
    time.sleep(0.5)

assert wait_for("[RTL] Climb complete. Returning to launch.", timeout=10), \
    "RTL climb never completed"

# Already at launch coords -- RTL_RETURN resolves on first nav tick
set_world(alt_ft=61.0)
assert wait_for("[RTL] Arrived over launch pad. Settling.", timeout=5), \
    "Never arrived over launch pad"

# RTL_SETTLE: 3s hover at cruise altitude
set_world(alt_ft=61.0)
assert wait_for("[RTL] Settle complete. Beginning landing.", timeout=8), \
    "Settle never completed"

# Landing from 61 ft at 1.5 ft/s -> ~41s
for alt in [55, 45, 35, 25, 15, 8, 3, 0]:
    set_world(alt_ft=float(alt))
    time.sleep(5.0)

assert wait_for("[NAV] Landed — motors disarmed.", timeout=15), \
    "Never landed"

# ── Whole-run FORBID checks ──────────────────────────────────────────────
forbid("[SAFETY] Geofence exceeded")   # timeout should fire before geofence
forbid("[PANIC]")
