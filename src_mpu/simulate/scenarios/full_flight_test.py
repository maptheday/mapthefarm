"""
Full Flight Regression Test
----------------------------
One continuous run: no-fix reject, arm, takeoff, e-stop mid-mission,
re-arm, full 4-waypoint mission, hover settle, auto-land.
"""
import time


# ── START ignored: no GPS fix yet ────────────────────────────────────────
send("CRSFSTART:1")
assert wait_for("[CRSF] START ignored -- no GPS fix.", timeout=5), \
    "Expected START to be rejected with no GPS fix"

# ── Establish initial position ───────────────────────────────────────────
set_world(lat=36.123456, lon=-80.123456, fix=True, alt_ft=0.0, heading_deg=90.0)
assert wait_for_gps_publish(), "GPS loop never published the initial fix"

# ── Arm + takeoff ────────────────────────────────────────────────────────
send("CRSFSTART:1")
assert wait_for("[CRSF] START switch -- arming and taking off.", timeout=5), \
    "Drone did not arm"

send("CRSFSTART:1")
assert wait_for("[CRSF] START ignored -- already flying/busy (phase: RAISE)", timeout=5), \
    "Expected START to be ignored mid-RAISE"

for alt in [3, 6, 10, 13, 15]:
    set_world(alt_ft=float(alt))
    time.sleep(1.0)

assert wait_for("[NAV] Takeoff altitude reached, transitioning to HOLD.", timeout=10), \
    "Never reached takeoff altitude"

# ── Start mission ────────────────────────────────────────────────────────
send("CRSFSTART:1")
assert wait_for("[CRSF] START switch -- starting waypoint mission.", timeout=5), \
    "Mission did not start"

for alt in [20, 30]:
    set_world(alt_ft=float(alt))
    time.sleep(0.5)

assert wait_for("[NAV] Reached waypoint 0 — advancing.", timeout=10), \
    "Never reached WP0"

# ── Move partway into WP0->WP1 then e-stop ──────────────────────────────
set_world(lat=36.123530, alt_ft=30.0)
assert wait_for_gps_publish(), "GPS loop never published the updated position"
send("CRSFSTOP:1")
assert wait_for("[CRSF] STOP switch -- emergency stop.", timeout=5), \
    "E-stop did not fire"

# Mission must NOT have completed on this first interrupted pass
forbid_count_gt_1("[NAV] Mission complete — hovering before landing.")

# ── Re-arm from PARKED ───────────────────────────────────────────────────
set_world(lat=36.123456, alt_ft=0.0, fix=True)
assert wait_for_gps_publish(), "GPS loop never published the updated position"

send("CRSFSTART:1")
assert wait_for("[CRSF] START switch -- arming and taking off.", timeout=5), \
    "Re-arm failed"

for alt in [3, 6, 10, 13, 15]:
    set_world(alt_ft=float(alt))
    time.sleep(1.0)

assert wait_for("[NAV] Takeoff altitude reached, transitioning to HOLD.", timeout=10), \
    "Never reached takeoff altitude on second arm"

send("CRSFSTART:1")
assert wait_for("[CRSF] START switch -- starting waypoint mission.", timeout=5), \
    "Second mission did not start"

for alt in [20, 30]:
    set_world(alt_ft=float(alt))
    time.sleep(0.5)

assert wait_for("[NAV] Reached waypoint 0 — advancing.", timeout=10), \
    "Never reached WP0 on second mission"

# ── Leg WP0 -> WP1 ──────────────────────────────────────────────────────
for lat in [36.123530, 36.123610]:
    set_world(lat=lat, alt_ft=30.0)
    assert wait_for_gps_publish(), "GPS loop never published the updated position"

# Wind gust: on real hardware, physically shake the board here.
# In HIL the IMU is live so any real disturbance will be felt by the firmware.
time.sleep(3.0)

set_world(lat=36.123690, alt_ft=30.0)
assert wait_for_gps_publish(), "GPS loop never published the updated position"

# Temporary GPS loss + recovery
set_world(fix=False, alt_ft=30.0)
assert wait_for_gps_publish(), "GPS loop never published the fix-lost state"
set_world(fix=True, alt_ft=30.0)
assert wait_for_gps_publish(), "GPS loop never published the recovered fix"

for lat in [36.123760, 36.123789]:
    set_world(lat=lat, alt_ft=30.0)
    assert wait_for_gps_publish(), "GPS loop never published the updated position"

assert wait_for("[NAV] Reached waypoint 1 — advancing.", timeout=15), \
    "Never reached WP1"

# ── Leg WP1 -> WP2 ──────────────────────────────────────────────────────
for lon in [-80.123530, -80.123610, -80.123690, -80.123760, -80.123789]:
    set_world(lon=lon, alt_ft=30.0)
    assert wait_for_gps_publish(), "GPS loop never published the updated position"

assert wait_for("[NAV] Reached waypoint 2 — advancing.", timeout=15), \
    "Never reached WP2"

# ── Leg WP2 -> WP3 ──────────────────────────────────────────────────────
for lat in [36.123710, 36.123630, 36.123550, 36.123480, 36.123456]:
    set_world(lat=lat, alt_ft=30.0)
    assert wait_for_gps_publish(), "GPS loop never published the updated position"

assert wait_for("[NAV] Reached waypoint 3 — advancing.", timeout=15), \
    "Never reached WP3"
assert wait_for("[NAV] Mission complete — hovering before landing.", timeout=5), \
    "Mission never completed"

# ── Hover settle (10s) ───────────────────────────────────────────────────
set_world(alt_ft=30.0)
assert wait_for("[NAV] Hover complete — beginning automatic landing.", timeout=15), \
    "Hover settle never completed"

# ── Landing: descend barometer as real sensor would ──────────────────────
# Firmware internal countdown from 30 ft at 1.5 ft/s -> ~20s
for alt in [25, 20, 15, 10, 5, 2, 0]:
    set_world(alt_ft=float(alt))
    time.sleep(3.0)

assert wait_for("[NAV] Landed — motors disarmed.", timeout=30), \
    "Never landed"

# ── Whole-run FORBID checks ──────────────────────────────────────────────
forbid("[PANIC]")
