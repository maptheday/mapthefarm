#pragma once

// Timing and safety limits. These are compile-time configuration values, not
// runtime state.
const unsigned long PHYSICS_LOOP_MS = 5;
const float         PHYSICS_LOOP_HZ = 1000.0f / PHYSICS_LOOP_MS;
const unsigned long NAV_LOOP_MS     = 100;

// Compass calibration: run it on boot? and how long to collect samples.
const bool          CALIBRATE_COMPASS_ON_BOOT = false;
const unsigned long COMPASS_CAL_DURATION_MS   = 30000;

// ESCs: standard PWM (see EspPwmESC.hpp).
// ESC calibration teaches the ESCs the throttle range (1000-2000 us). Do it once
// when the ESCs are new: set true, flash, follow the prompts in the serial
// monitor with PROPS OFF, then set back to false and flash again. While true,
// the drone never flies -- it halts after calibrating.
const bool CALIBRATE_ESCS_ON_BOOT = false;
// Pulses per second sent to the ESCs. 400 suits multirotor ESCs (SimonK,
// BLHeli). If the motors stutter or won't arm, some plane ESCs only accept 50.
const int  ESC_PWM_HZ             = 400;

// Bench test: a props-OFF checkout of the finished drone (see BenchTest.hpp).
// Set true, flash, and use the serial monitor to spin each motor and watch the
// sensors. While true, the drone never flies. Set back to false when done.
const bool BENCH_TEST_ON_BOOT     = false;

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
// ---- Battery monitoring (see Battery.hpp, modeled on INAV's battery.c) -------
// The flight pack: 3S LiFePO4, 2100 mAh (Pulse). Voltages below are PER CELL.
const int   BATTERY_CELLS         = 3;
const float BATTERY_CAPACITY_MAH  = 2100.0f;
// Capacity thresholds (the main gauge for LiFe, whose voltage is flat until the end).
// WARNING or CRITICAL -> land where it is. Landing from 30 ft takes ~20 s, well
// inside the 10% left between the two.
const float BATTERY_WARNING_USED  = 0.70f;   // estimated 70% used
const float BATTERY_CRITICAL_USED = 0.80f;   // estimated 80% used (the 80% rule)
// Voltage thresholds (the backup, catches a pack that wasn't full or a bad estimate).
// LiFe values, measured under load after the 1 Hz filter. Starting guesses: tune
// them from real flight logs.
const float CELL_WARNING_V        = 3.00f;
const float CELL_CRITICAL_V       = 2.85f;
const float CELL_TAKEOFF_MIN_V    = 3.25f;   // at rest; START is refused below this
// Voltage divider: battery+ --[100k]--+--[22k]-- GND, junction -> BATTERY_ADC_PIN.
// It shrinks the ~11 V pack to ~2 V, which the ESP32 can measure.
#define     BATTERY_ADC_PIN         1
const float BATTERY_DIVIDER_SCALE = 5.545f;  // (100k + 22k) / 22k. Fine-tune with a multimeter.
// Virtual current sensor (INAV's idea): no current sensor on this drone, so the
// current is estimated from the motor commands. A propeller's power grows with
// speed cubed, so:  amps ≈ IDLE + MOTOR_FULL × (m1³ + m2³ + m3³ + m4³).
// Tune CURRENT_MOTOR_FULL_A until the estimated mAh matches what the charger puts back.
const float CURRENT_IDLE_A        = 0.3f;    // electronics only, motors stopped
const float CURRENT_MOTOR_FULL_A  = 16.0f;   // one motor at full throttle

// Radio link loss: no radio frame for this long -> come home (see Failsafes.hpp).
const unsigned long RC_LOSS_TIMEOUT_MS  = 1000;
// ...unless we're this low (or have no GPS): then land in place instead, so a
// link drop on the ground never starts a climb.
const float         RC_LOSS_LAND_BELOW_FT = 5.0f;
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