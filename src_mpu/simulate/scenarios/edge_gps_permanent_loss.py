"""
Edge Case: Permanent GPS Loss
------------------------------
Arm, take off, kill GPS fix and never restore it. Firmware should abort
directly to LANDING after GPS_LOSS_ABORT_MS=3000ms, skipping RTL entirely
(can't navigate home without a position fix).

Landing from 15 ft at 1.5 ft/s -> ~10s.
"""
import time


set_world(lat=36.123456, lon=-80.123456, fix=True, alt_ft=0.0, heading_deg=90.0)
assert wait_for_gps_publish(), "GPS loop never published the initial fix"

send("CRSFSTART:1")
assert wait_for("[CRSF] START switch -- arming and taking off.", timeout=5), \
    "Drone did not arm"

for alt in [3, 6, 10, 13, 15]:
    set_world(alt_ft=float(alt))
    time.sleep(1.0)

assert wait_for("[NAV] Takeoff altitude reached, transitioning to HOLD.", timeout=10), \
    "Never reached takeoff altitude"

# Kill GPS fix, never restore. Keep barometer and compass honest.
set_world(fix=False, alt_ft=15.0)

assert wait_for("[SAFETY] GPS fix lost — aborting directly to LANDING.", timeout=8), \
    "GPS loss was not detected within timeout"

# Landing from 15 ft at 1.5 ft/s -> ~10s
for alt in [12, 9, 6, 3, 0]:
    set_world(alt_ft=float(alt))
    time.sleep(2.0)

assert wait_for("[NAV] Landed — motors disarmed.", timeout=15), \
    "Never landed"

# ── Whole-run FORBID checks ──────────────────────────────────────────────
forbid("[RTL]")     # must go straight to LANDING, never through RTL
forbid("[PANIC]")
