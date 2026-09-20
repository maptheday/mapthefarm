#pragma once

// Timing and safety limits. These are compile-time configuration values, not
// runtime state.
const unsigned long PHYSICS_LOOP_MS = 5;
const float         PHYSICS_LOOP_HZ = 1000.0f / PHYSICS_LOOP_MS;
const unsigned long NAV_LOOP_MS     = 100;

// Compass calibration: run it on boot? and how long to collect samples.
const bool          CALIBRATE_COMPASS_ON_BOOT = false;
const unsigned long COMPASS_CAL_DURATION_MS   = 30000;

#if defined(SIM)
// Mutable in sim so a test scenario can shorten the limit to trip a failsafe on
// purpose (e.g. the timeout / geofence scenarios). On real hardware they're const.
unsigned long MAX_FLIGHT_TIME_MS = 10UL * 60UL * 1000UL;  // room to fly the whole perimeter
float         GEOFENCE_RADIUS_M  = 450.0f;               // must enclose the whole field
#else
const unsigned long MAX_FLIGHT_TIME_MS = 5UL * 60UL * 1000UL;
const float         GEOFENCE_RADIUS_M  = 450.0f;   // must enclose the whole field
#endif
const unsigned long GPS_LOSS_ABORT_MS   = 3000;
const float         RTL_ALTITUDE_FT     = 60.0f;
const unsigned long RTL_SETTLE_MS       = 3000;   // hover this long over launch before landing
// Hover throttle feed-forward: the altitude PID only trims deviations around
// this baseline instead of winding its integral all the way up from zero, which
// is what made the height hunt up and down. Tune per airframe (roughly the
// throttle it takes to hover; ~0.5 for a normal ~2:1 thrust/weight craft).
const float         HOVER_THROTTLE_FF   = 0.50f;
const float         TAKEOFF_ALTITUDE_FT = 15.0f;

// Mission configuration.
struct Waypoint {
  double lat;
  double lon;
  float  altFt;
};

// The farmer's real field perimeter (captured GPS corners). Launch is at
// corner 1; the mission flies corners 2..8 and then back to corner 1, tracing
// the whole fence line (~765 m, up to ~300 m from launch) before landing.
const Waypoint WAYPOINTS[] = {
  { 35.947693, -78.240760, 30.0f },  // corner 2
  { 35.947806, -78.239370, 30.0f },  // corner 3
  { 35.947923, -78.239510, 30.0f },  // corner 4
  { 35.948327, -78.238051, 30.0f },  // corner 5
  { 35.949061, -78.238292, 30.0f },  // corner 6
  { 35.948587, -78.240529, 30.0f },  // corner 7
  { 35.948500, -78.240432, 30.0f },  // corner 8
  { 35.948305, -78.241377, 30.0f },  // back to corner 1 (launch)
};
const int   WAYPOINT_COUNT = sizeof(WAYPOINTS) / sizeof(WAYPOINTS[0]);
const float WAYPOINT_ACCEPT_RADIUS_M = 4.0f;   // reach each corner tightly -> straight legs, sharp turns
// Line-following lookahead: steer toward a point this far ahead along the current
// leg (previous waypoint -> current waypoint) so the drone hugs the straight line
// instead of cutting to the inside of the turn. Smaller = tighter tracking but
// twitchier; larger = smoother but cuts corners more.
const float WAYPOINT_LOOKAHEAD_M     = 18.0f;
const unsigned long MISSION_COMPLETE_HOVER_MS = 10000;
const float LAND_DESCENT_RATE_FPS = 1.5f;
const float RAISE_CLIMB_RATE_FPS = 3.0f;

// GPS (BN-880) serial wiring.
#define GPS_RX_PIN 17
#define GPS_TX_PIN 18
#define GPS_BAUD   9600

// CRSF / ELRS radio input configuration.
#define CRSF_RX_PIN         8
#define CRSF_BAUD           420000
#define CRSF_START_CH       4
#define CRSF_STOP_CH        5
#define CRSF_HIGH_THRESHOLD 1700
#define CRSF_LOW_THRESHOLD  1300

// MANUAL mode: stick channels (0-based, standard AETR order) + the AUX switch
// that flips into/out of pilot control. Raw CRSF values are 172..1811 (mid 992).
#define CRSF_ROLL_CH        0
#define CRSF_PITCH_CH       1
#define CRSF_THROTTLE_CH    2
#define CRSF_YAW_CH         3
#define CRSF_MANUAL_CH      6   // 3-pos switch: HIGH = take manual control
#define CRSF_RAW_MIN        172
#define CRSF_RAW_MID        992
#define CRSF_RAW_MAX        1811

// How far the sticks push the setpoints at full deflection.
const float MANUAL_MAX_LEAN_DEG   = 15.0f;  // full roll/pitch stick -> 15 deg lean
const float MANUAL_CLIMB_RATE_FPS = 3.0f;   // full up/down throttle -> 3 ft/s climb/descend
const float MANUAL_YAW_RATE_DPS   = 45.0f;  // full yaw stick -> 45 deg/s turn
const float MANUAL_STICK_DEADBAND = 0.05f;  // ignore tiny stick noise near center

// Position hold: when you center the roll/pitch sticks, actively brake and hold
// the GPS spot instead of coasting. Needs a real GPS fix. Set to false for
// bench/indoor testing -- then a centered stick just levels out (as before).
const bool MANUAL_POSITION_HOLD = true;