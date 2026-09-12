#pragma once

// Timing and safety limits. These are compile-time configuration values, not
// runtime state.
const unsigned long PHYSICS_LOOP_MS = 5;
const float         PHYSICS_LOOP_HZ = 1000.0f / PHYSICS_LOOP_MS;
const unsigned long NAV_LOOP_MS     = 100;

// Compass calibration: run it on boot? and how long to collect samples.
const bool          CALIBRATE_COMPASS_ON_BOOT = false;
const unsigned long COMPASS_CAL_DURATION_MS   = 30000;

#ifdef WOKWI_SIM
const unsigned long MAX_FLIGHT_TIME_MS = 60UL * 1000UL;
#else
const unsigned long MAX_FLIGHT_TIME_MS = 5UL * 60UL * 1000UL;
#endif

const float         GEOFENCE_RADIUS_M   = 150.0f;
const unsigned long GPS_LOSS_ABORT_MS   = 3000;
const float         RTL_ALTITUDE_FT     = 60.0f;
const float         TAKEOFF_ALTITUDE_FT = 15.0f;

// Mission configuration.
struct Waypoint {
  double lat;
  double lon;
  float  altFt;
};

const Waypoint WAYPOINTS[] = {
  { 36.123456, -80.123456, 30.0f },
  { 36.123789, -80.123456, 30.0f },
  { 36.123789, -80.123789, 30.0f },
  { 36.123456, -80.123789, 30.0f },
};
const int   WAYPOINT_COUNT = sizeof(WAYPOINTS) / sizeof(WAYPOINTS[0]);
const float WAYPOINT_ACCEPT_RADIUS_M = 5.0f;
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