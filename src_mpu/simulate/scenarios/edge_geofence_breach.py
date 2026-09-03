"""
Edge Case: Geofence Breach
---------------------------
Arm, take off, teleport GPS ~400m north (outside GEOFENCE_RADIUS_M=150m),
confirm forced RTL, climb, return to launch, settle, land.
"""
import time

# assert wait_for("[WEB] Server started.", timeout=30), "Firmware never booted"

set_world(lat=36.123456, lon=-80.123456, fix=True, alt_ft=0.0, heading_deg=90.0)

# Allow the 1 Hz _gps_loop thread to transmit FIX:1, LAT, and LON to firmware
time.sleep(1.2)

send("CRSFSTART:1")
assert wait_for("[CRSF] START switch -- arming and taking off.", timeout=5), \
    "Drone did not arm"

for alt in [3, 6, 10, 13, 15]:
    set_world(alt_ft=float(alt))
    time.sleep(1.0)

assert wait_for("[NAV] Takeoff altitude reached, transitioning to HOLD.", timeout=10), \
    "Never reached takeoff altitude"

# Teleport GPS ~400m north -- well outside geofence
set_world(lat=36.127056, alt_ft=15.0)
assert wait_for("[SAFETY] Geofence exceeded — forcing RTL.", timeout=5), \
    "Geofence breach was not detected"

# RTL_CLIMB: ramp barometer up to RTL_ALTITUDE_FT (60 ft)
for alt in [20, 30, 40, 50, 61]:
    set_world(alt_ft=float(alt))
    time.sleep(0.5)

assert wait_for("[RTL] Climb complete. Returning to launch.", timeout=10), \
    "RTL climb never completed"

# Walk GPS back to launch. Keep barometer at cruise altitude.
for lat in [36.125000, 36.123500, 36.123456]:
    set_world(lat=lat, alt_ft=61.0)
    time.sleep(0.5)

assert wait_for("[RTL] Arrived over launch pad. Settling.", timeout=10), \
    "Never arrived over launch pad"

# RTL_SETTLE: 3s hover. Keep barometer at 61 ft so transitionTo(PHASE_LANDING)
# snapshots the real cruise altitude, not zero.
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
forbid("[NAV] Reached waypoint")   # mission was never started
forbid("[PANIC]")
