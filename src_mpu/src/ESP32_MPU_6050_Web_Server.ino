#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
#include <MadgwickAHRS.h>
#include "LittleFS.h"
#include <TinyGPSPlus.h>
#include <QMC5883LCompass.h>
#include <Preferences.h>

#include "EspBarometer.hpp"
#include "EspESC.hpp"
#include "PID.hpp"

const char* ssid     = "Wokwi-GUEST";
const char* password = "";

// ==========================================
// TIMING CONFIGURATION
// ==========================================
const unsigned long PHYSICS_LOOP_MS = 5;
const float         PHYSICS_LOOP_HZ = 1000.0f / PHYSICS_LOOP_MS;
const unsigned long NAV_LOOP_MS     = 100;

const unsigned long SSE_GYRO_MS     = 10;
const unsigned long SSE_ACC_MS      = 200;
const unsigned long SSE_FLIGHT_MS   = 100;

// ==========================================
// SAFETY CONFIGURATION
// ==========================================
const unsigned long MAX_FLIGHT_TIME_MS  = 5UL * 60UL * 1000UL;
const float         GEOFENCE_RADIUS_M   = 150.0f;
const unsigned long GPS_LOSS_ABORT_MS   = 3000;
const float         RTL_ALTITUDE_FT     = 60.0f;
const float         TAKEOFF_ALTITUDE_FT = 15.0f;

// ==========================================
// BN-880 WIRING
// ==========================================
#define GPS_RX_PIN 17
#define GPS_TX_PIN 18
#define GPS_BAUD   9600

// ==========================================
// COMPASS CALIBRATION
// ==========================================
const bool          CALIBRATE_COMPASS_ON_BOOT = false;
const unsigned long COMPASS_CAL_DURATION_MS   = 30000;

Preferences compassPrefs;
float compassOffsetX = 0;
float compassOffsetY = 0;
float compassOffsetZ = 0;
float compassScaleX  = 1;
float compassScaleY  = 1;
float compassScaleZ  = 1;

// ==========================================
// ESC CALIBRATION
// ==========================================
// Runs every boot: full-throttle → min-throttle → arm.
// ESCs save the throttle range to their own EEPROM each time.
// PWM pulse widths (microseconds) — adjust to match your ESC spec.
const int   ESC_PULSE_MAX_US = 2000;
const int   ESC_PULSE_MIN_US = 1000;
const int   ESC_PULSE_ARM_US = 1000; // Arm signal = min throttle

// ==========================================
// WAYPOINT SYSTEM
// ==========================================
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
const int   WAYPOINT_COUNT            = sizeof(WAYPOINTS) / sizeof(WAYPOINTS[0]);
const float WAYPOINT_ACCEPT_RADIUS_M  = 5.0f;
const unsigned long MISSION_COMPLETE_HOVER_MS = 10000;
const float LAND_DESCENT_RATE_FPS     = 1.5f;
const float RAISE_CLIMB_RATE_FPS      = 3.0f;

// ==========================================
// CRSF / ELRS RC INPUT
// ==========================================
// Wire ELRS receiver TX pin to ESP32 GPIO 16.
// Bind receiver to your EdgeTX radio before first flight.
// CRSF baud rate is always 420000.
//
// Two-switch layout (assign in EdgeTX mixer):
//
//   Ch5 (AUX1) -- START switch
//     Flip UP  (value > 1700) -> Begins takeoff (if parked) OR starts mission (if holding)
//     Flip DOWN (value < 1300) -> No action (idle)
//
//   Ch6 (AUX2) -- STOP switch
//     Flip DOWN (value < 1300) -> EMERGENCY STOP, motors cut instantly
//     Flip UP   (value > 1700) -> No action (idle/reset)
//
// START is edge-triggered (fires only on the LOW->HIGH transition) so
// holding the switch up does not repeatedly re-arm or re-trigger.
// STOP is level-triggered: any frame with Ch6 below threshold kills motors.
//
#define CRSF_RX_PIN         16
#define CRSF_BAUD           420000
#define CRSF_START_CH       4     // Zero-indexed (Ch5 on radio)
#define CRSF_STOP_CH        5     // Zero-indexed (Ch6 on radio)
#define CRSF_HIGH_THRESHOLD 1700  // microseconds -- above this = switch UP
#define CRSF_LOW_THRESHOLD  1300  // microseconds -- below this = switch DOWN

// ==========================================
// WOKWI SIMULATION STATE
// ==========================================
#ifdef WOKWI_SIM
volatile float  simCompassHeading = 0.0f;
volatile double simGpsLat         = 36.123456;
volatile double simGpsLon         = -80.123456;
volatile bool   simGpsFix         = false;
volatile bool   simStartMission   = false;
#endif

// ==========================================
// HARDWARE
// ==========================================
Adafruit_MPU6050 mpu;
Madgwick         filter;
EspBarometer     barometer;
TinyGPSPlus      gps;
QMC5883LCompass  compass;

AsyncWebServer    server(80);

// ==========================================
// PID CONTROLLERS
// ==========================================
//
// All values are first-guess starting points, NOT flight-ready.
// Tune on a tether -- expect to revise these significantly.
//
// Quick field guide:
//   Too much P  -> fast oscillation (buzzing)
//   Too little P -> sluggish, drifts
//   Too much I  -> slow-growing oscillation, integrator windup
//   Too much D  -> high-frequency jitter, noise amplification
//
// Suggested tether tuning order:
//   1. altitudePID  -- stable hover height
//   2. rollPID / pitchPID -- level attitude, no toilet-bowl spin
//   3. yawPID -- nose holds heading without hunting
//   4. navNorthPID / navEastPID -- gentle waypoint tracking last
//
PID altitudePID (0.08f,  0.01f,   0.05f,  0.0f,   1.0f  );
PID rollPID     (0.01f,  0.001f,  0.005f, -0.3f,  0.3f  );
PID pitchPID    (0.01f,  0.001f,  0.005f, -0.3f,  0.3f  );
PID yawPID      (0.005f, 0.0001f, 0.001f, -0.2f,  0.2f  );
PID navNorthPID (0.5f,   0.0f,    0.1f,   -15.0f, 15.0f );
PID navEastPID  (0.5f,   0.0f,    0.1f,   -15.0f, 15.0f );

// ==========================================
// PANIC
// ==========================================
#define PANIC(msg) do { Serial.println(F("[PANIC] " msg " — halting")); while(1) { delay(10); } } while(0)

#define ASSERT_MOTORS_INITIALIZED(d) \
  do { if ((d).m1 == 0.0f && (d).m2 == 0.0f && (d).m3 == 0.0f && (d).m4 == 0.0f) \
    PANIC("control decision on uninitialized motor outputs"); } while(0)

// ==========================================
// FLIGHT PHASE ENUM
// ==========================================
enum FlightPhase {
  PHASE_PARKED,
  PHASE_RAISE,
  PHASE_HOLD,
  PHASE_MISSION,
  PHASE_RTL,
  PHASE_HOVER_SETTLE,
  PHASE_LANDING,
  PHASE_LANDED
};

const char* phaseName(FlightPhase p) {
  switch (p) {
    case PHASE_PARKED:       return "PARKED";
    case PHASE_RAISE:        return "RAISE";
    case PHASE_HOLD:         return "HOLD";
    case PHASE_MISSION:      return "MISSION";
    case PHASE_RTL:          return "RTL";
    case PHASE_HOVER_SETTLE: return "HOVER_SETTLE";
    case PHASE_LANDING:      return "LANDING";
    case PHASE_LANDED:       return "LANDED";
  }
  PANIC("phaseName: unhandled FlightPhase");
  return nullptr;
}

bool phaseFlightEnabled(FlightPhase phase) {
  return phase != PHASE_PARKED && phase != PHASE_LANDED;
}

// ==========================================
// RAW SENSOR READS
// ==========================================
struct RawImuReading {
  float accX  = 0.0f;
  float accY  = 0.0f;
  float accZ  = 0.0f;
  // NOTE: gyroX/Y/Z are overwritten in physicsTask with Madgwick filter
  // angles (roll/pitch/yaw in degrees), NOT raw gyro rates.
  // The field names are kept for historical reasons.
  float gyroX = 0.0f; // Madgwick roll  (deg)
  float gyroY = 0.0f; // Madgwick pitch (deg)
  float gyroZ = 0.0f; // Madgwick yaw   (deg)
  float temp  = 0.0f;
};

struct RawGpsReading {
  double lat = 0.0;
  double lon = 0.0;
  bool   fix = false;
  int    sats = 0;
  float  speedMps = 0.0f;
  unsigned long lastFixMs = 0;
};

struct RawSensors {
  RawImuReading imu;
  RawGpsReading gps;
  float         compassHeadingDeg = 0.0f;
  float         baroAltitudeFt    = 0.0f; 
};

// ==========================================
// PER-PHASE STATE BLOCKS
// ==========================================

// --- PARKED ---
struct Dashboard_Parked {
  float altitudeFt = 0.0f;
  float roll       = 0.0f;
  float pitch      = 0.0f;
  float yaw        = 0.0f;
};
struct Cruise_Parked {};
struct Trip_Parked {};

// --- RAISE (Takeoff) ---
struct Dashboard_Raise {
  float altitudeFt      = 0.0f;
  float roll            = 0.0f;
  float pitch           = 0.0f;
  float yaw             = 0.0f;
  float compassHeading  = 0.0f;
  float m1              = 0.0f;
  float m2              = 0.0f;
  float m3              = 0.0f;
  float m4              = 0.0f;
  float baseThrottle    = 0.0f;
  float rollCorrection  = 0.0f;
  float pitchCorrection = 0.0f;
};
struct Cruise_Raise {
  float targetAltFt      = 0.0f;
  float targetRollDeg    = 0.0f;
  float targetPitchDeg   = 0.0f;
  float yawTargetHeading = 0.0f;
};
struct Trip_Raise {
  unsigned long armedAtMs = 0;
  double        launchLat = 0.0;
  double        launchLon = 0.0;
};

// --- HOLD ---
struct Dashboard_Hold {
  float altitudeFt      = 0.0f;
  float roll            = 0.0f;
  float pitch           = 0.0f;
  float yaw             = 0.0f;
  float compassHeading  = 0.0f;
  float m1              = 0.0f;
  float m2              = 0.0f;
  float m3              = 0.0f;
  float m4              = 0.0f;
  float baseThrottle    = 0.0f;
  float rollCorrection  = 0.0f;
  float pitchCorrection = 0.0f;
};
struct Cruise_Hold {
  float targetAltFt      = 10.0f;
  float targetRollDeg    = 0.0f;
  float targetPitchDeg   = 0.0f;
  float yawTargetHeading = 0.0f;
};
struct Trip_Hold {
  unsigned long armedAtMs = 0;
  double        launchLat = 0.0;
  double        launchLon = 0.0;
};

// --- MISSION ---
struct Dashboard_Mission {
  float  altitudeFt      = 0.0f;
  float  roll            = 0.0f;
  float  pitch           = 0.0f;
  float  yaw             = 0.0f;
  float  compassHeading  = 0.0f;
  double gpsLat          = 0.0;
  double gpsLon          = 0.0;
  bool   gpsFix          = false;
  int    gpsSats         = 0;
  float  distToWP        = 0.0f;
  float  bearingToWP     = 0.0f;
  float  m1              = 0.0f;
  float  m2              = 0.0f;
  float  m3              = 0.0f;
  float  m4              = 0.0f;
  float  baseThrottle    = 0.0f;
  float  rollCorrection  = 0.0f;
  float  pitchCorrection = 0.0f;
};
struct Cruise_Mission {
  float targetAltFt      = 10.0f;
  float targetRollDeg    = 0.0f;
  float targetPitchDeg   = 0.0f;
  float yawTargetHeading = 0.0f;
};
struct Trip_Mission {
  unsigned long armedAtMs     = 0;
  int           currentWP     = 0;
  int           waypointCount = 0;
  bool          active        = false;
  double        launchLat     = 0.0;
  double        launchLon     = 0.0;
};

// --- RTL ---
enum RTLState {
  RTL_CLIMB,
  RTL_RETURN,
  RTL_SETTLE
};

struct Dashboard_RTL {
  float altitudeFt      = 0.0f;
  float roll            = 0.0f;
  float pitch           = 0.0f;
  float yaw             = 0.0f;
  float compassHeading  = 0.0f;
  float m1              = 0.0f;
  float m2              = 0.0f;
  float m3              = 0.0f;
  float m4              = 0.0f;
  float baseThrottle    = 0.0f;
  float rollCorrection  = 0.0f;
  float pitchCorrection = 0.0f;
};
struct Cruise_RTL {
  float targetAltFt      = 10.0f;
  float targetRollDeg    = 0.0f;
  float targetPitchDeg   = 0.0f;
  float yawTargetHeading = 0.0f;
};
struct Trip_RTL {
  unsigned long armedAtMs     = 0;
  RTLState      state         = RTL_CLIMB;
  double        launchLat     = 0.0;
  double        launchLon     = 0.0;
  unsigned long settleStartMs = 0;
};

// --- HOVER SETTLE ---
struct Dashboard_HoverSettle {
  float altitudeFt      = 0.0f;
  float roll            = 0.0f;
  float pitch           = 0.0f;
  float yaw             = 0.0f;
  float compassHeading  = 0.0f;
  float m1              = 0.0f;
  float m2              = 0.0f;
  float m3              = 0.0f;
  float m4              = 0.0f;
  float baseThrottle    = 0.0f;
  float rollCorrection  = 0.0f;
  float pitchCorrection = 0.0f;
};
struct Cruise_HoverSettle {
  float targetAltFt      = 10.0f;
  float targetRollDeg    = 0.0f;
  float targetPitchDeg   = 0.0f;
  float yawTargetHeading = 0.0f;
};
struct Trip_HoverSettle {
  unsigned long enteredAtMs = 0;
};

// --- LANDING ---
struct Dashboard_Landing {
  float altitudeFt      = 0.0f;
  float roll            = 0.0f;
  float pitch           = 0.0f;
  float yaw             = 0.0f;
  float compassHeading  = 0.0f;
  float m1              = 0.0f;
  float m2              = 0.0f;
  float m3              = 0.0f;
  float m4              = 0.0f;
  float baseThrottle    = 0.0f;
  float rollCorrection  = 0.0f;
  float pitchCorrection = 0.0f;
};
struct Cruise_Landing {
  float targetAltFt      = 0.0f;
  float targetRollDeg    = 0.0f;
  float targetPitchDeg   = 0.0f;
  float yawTargetHeading = 0.0f;
};
struct Trip_Landing {};

// --- LANDED ---
struct Dashboard_Landed {
  float altitudeFt = 0.0f;
};
struct Cruise_Landed {};
struct Trip_Landed {};

// ==========================================
// SHARED STATE
// ==========================================
struct SharedState {
  FlightPhase phase = PHASE_PARKED;
  RawSensors  raw;

  Dashboard_Parked      dashboard_parked;
  Cruise_Parked         cruise_parked;
  Trip_Parked           trip_parked;

  Dashboard_Raise       dashboard_raise;
  Cruise_Raise          cruise_raise;
  Trip_Raise            trip_raise;

  Dashboard_Hold        dashboard_hold;
  Cruise_Hold           cruise_hold;
  Trip_Hold             trip_hold;

  Dashboard_Mission     dashboard_mission;
  Cruise_Mission        cruise_mission;
  Trip_Mission          trip_mission;

  Dashboard_RTL         dashboard_rtl;
  Cruise_RTL            cruise_rtl;
  Trip_RTL              trip_rtl;

  Dashboard_HoverSettle dashboard_hoverSettle;
  Cruise_HoverSettle    cruise_hoverSettle;
  Trip_HoverSettle      trip_hoverSettle;

  Dashboard_Landing     dashboard_landing;
  Cruise_Landing        cruise_landing;
  Trip_Landing          trip_landing;

  Dashboard_Landed      dashboard_landed;
  Cruise_Landed         cruise_landed;
  Trip_Landed           trip_landed;
};

SemaphoreHandle_t sharedDataMutex;
SemaphoreHandle_t serialMutex;

volatile SharedState shared;
float groundAltitudeFt = 0;

template<typename Fn>
void withMutex(Fn fn) {
  if (xSemaphoreTake(sharedDataMutex, portMAX_DELAY) == pdTRUE) {
    fn();
    xSemaphoreGive(sharedDataMutex);
  }
}

void logLine(const String& msg) {
  if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    Serial.println(msg);
    xSemaphoreGive(serialMutex);
  }
}

// ==========================================
// GPS / NAVIGATION MATH HELPERS
// ==========================================
float gpsDistanceMeters(double lat1, double lon1, double lat2, double lon2) {
  const float R = 6371000.0f;
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  float a = sin(dLat/2)*sin(dLat/2) + cos(radians(lat1))*cos(radians(lat2))*sin(dLon/2)*sin(dLon/2);
  return R * 2.0f * atan2(sqrt(a), sqrt(1.0f - a));
}

float gpsBearing(double lat1, double lon1, double lat2, double lon2) {
  float dLon = radians(lon2 - lon1);
  float y    = sin(dLon) * cos(radians(lat2));
  float x    = cos(radians(lat1)) * sin(radians(lat2)) - sin(radians(lat1)) * cos(radians(lat2)) * cos(dLon);
  return fmod(degrees(atan2(y, x)) + 360.0f, 360.0f);
}

void bearingToNorthEast(float distM, float bearingDeg, float& northM, float& eastM) {
  float rad = radians(bearingDeg);
  northM = distM * cos(rad);
  eastM  = distM * sin(rad);
}

Waypoint getMissionWaypoint(int index, double launchLat, double launchLon) {
  if (index < WAYPOINT_COUNT) return WAYPOINTS[index];
  float holdAlt = WAYPOINT_COUNT > 0 ? WAYPOINTS[WAYPOINT_COUNT - 1].altFt : 10.0f;
  return { launchLat, launchLon, holdAlt };
}

// ==========================================
// MOTOR MIX HELPER
// ==========================================
struct MotorMix {
  float m1;
  float m2;
  float m3;
  float m4;
  float baseThrottle;
  float rollCorrection;
  float pitchCorrection;
};

MotorMix computeMotorMix(float targetAltFt, float targetRollDeg, float targetPitchDeg,
                          float yawTargetHeading, float altFt, float roll, float pitch, 
                          float compassHeading, float gyroZ, float dt) {
  MotorMix out;
  out.baseThrottle    = altitudePID.compute(targetAltFt,    altFt,         dt);
  out.rollCorrection  = rollPID.compute    (targetRollDeg,  roll,          dt);
  out.pitchCorrection = pitchPID.compute   (targetPitchDeg, pitch,         dt);

  float yawError = yawTargetHeading - compassHeading;
  if (yawError >  180.0f) yawError -= 360.0f;
  if (yawError < -180.0f) yawError += 360.0f;
  
  // Base compass correction
  float baseYawCorr = yawPID.computeWithError(yawError, dt);
  
  // Instantaneous gyroZ damping reflex
  const float YAW_RATE_DAMPING = 0.02f; 
  float yawCorr = baseYawCorr - (gyroZ * YAW_RATE_DAMPING);

  out.m1 = constrain(out.baseThrottle + out.pitchCorrection + out.rollCorrection - yawCorr, 0.0f, 1.0f);
  out.m2 = constrain(out.baseThrottle + out.pitchCorrection - out.rollCorrection + yawCorr, 0.0f, 1.0f);
  out.m3 = constrain(out.baseThrottle - out.pitchCorrection + out.rollCorrection + yawCorr, 0.0f, 1.0f);
  out.m4 = constrain(out.baseThrottle - out.pitchCorrection - out.rollCorrection - yawCorr, 0.0f, 1.0f);
  return out;
}

// ==========================================
// FUNCTIONAL STATE SETTERS / TRANSITIONS
// ==========================================
void transitionTo(FlightPhase next) {
  withMutex([&]() {
    FlightPhase prev     = shared.phase;
    float currentAlt     = shared.raw.baroAltitudeFt;
    float currentHeading = shared.raw.compassHeadingDeg;
    unsigned long now    = millis();
    double currentLat    = shared.raw.gps.lat;
    double currentLon    = shared.raw.gps.lon;

    // Track original arming time and launch coords to persist across flight phases
    unsigned long activeArmedAt = 0;
    double activeLaunchLat      = 0.0;
    double activeLaunchLon      = 0.0;
    
    if (prev == PHASE_RAISE) { 
      activeArmedAt   = shared.trip_raise.armedAtMs;     
      activeLaunchLat = shared.trip_raise.launchLat;     
      activeLaunchLon = shared.trip_raise.launchLon; 
    }
    if (prev == PHASE_HOLD) { 
      activeArmedAt   = shared.trip_hold.armedAtMs;      
      activeLaunchLat = shared.trip_hold.launchLat;      
      activeLaunchLon = shared.trip_hold.launchLon; 
    }
    if (prev == PHASE_MISSION) { 
      activeArmedAt   = shared.trip_mission.armedAtMs;   
      activeLaunchLat = shared.trip_mission.launchLat;   
      activeLaunchLon = shared.trip_mission.launchLon; 
    }
    if (prev == PHASE_RTL) { 
      activeArmedAt   = shared.trip_rtl.armedAtMs;       
      activeLaunchLat = shared.trip_rtl.launchLat;       
      activeLaunchLon = shared.trip_rtl.launchLon; 
    }

    switch (next) {
      case PHASE_PARKED: break;

      case PHASE_RAISE:
        // Reset all PID integrators so stale state from a previous flight
        // cannot corrupt the first takeoff ramp.
        altitudePID.reset(); rollPID.reset(); pitchPID.reset();
        yawPID.reset(); navNorthPID.reset(); navEastPID.reset();
        shared.trip_raise.armedAtMs          = now;
        shared.trip_raise.launchLat          = currentLat;
        shared.trip_raise.launchLon          = currentLon;
        shared.cruise_raise.targetAltFt      = 0.0f; // Start ramp from 0
        shared.cruise_raise.yawTargetHeading = currentHeading;
        shared.cruise_raise.targetRollDeg    = 0.0f;
        shared.cruise_raise.targetPitchDeg   = 0.0f;
        shared.dashboard_raise.m1            = 0.0f;
        shared.dashboard_raise.m2            = 0.0f;
        shared.dashboard_raise.m3            = 0.0f;
        shared.dashboard_raise.m4            = 0.0f;
        break;

      case PHASE_HOLD:
        shared.trip_hold.armedAtMs           = activeArmedAt;
        shared.trip_hold.launchLat           = activeLaunchLat;
        shared.trip_hold.launchLon           = activeLaunchLon;
        shared.cruise_hold.targetAltFt       = (prev == PHASE_RAISE) ? TAKEOFF_ALTITUDE_FT : currentAlt; 
        shared.cruise_hold.yawTargetHeading  = currentHeading;
        shared.cruise_hold.targetRollDeg     = 0.0f;
        shared.cruise_hold.targetPitchDeg    = 0.0f;
        shared.dashboard_hold.m1             = 0.0f;
        shared.dashboard_hold.m2             = 0.0f;
        shared.dashboard_hold.m3             = 0.0f;
        shared.dashboard_hold.m4             = 0.0f;
        break;

      case PHASE_MISSION:
        if (WAYPOINT_COUNT == 0) { PANIC("transitionTo MISSION: zero waypoints"); }
        shared.trip_mission.armedAtMs        = activeArmedAt;
        shared.trip_mission.launchLat        = activeLaunchLat;
        shared.trip_mission.launchLon        = activeLaunchLon;
        shared.trip_mission.currentWP        = 0;
        shared.trip_mission.waypointCount    = WAYPOINT_COUNT; 
        shared.trip_mission.active           = true;
        shared.cruise_mission.targetAltFt    = WAYPOINTS[0].altFt;
        shared.cruise_mission.yawTargetHeading = currentHeading;
        shared.cruise_mission.targetRollDeg  = 0.0f;
        shared.cruise_mission.targetPitchDeg = 0.0f;
        shared.dashboard_mission.m1          = 0.0f;
        shared.dashboard_mission.m2          = 0.0f;
        shared.dashboard_mission.m3          = 0.0f;
        shared.dashboard_mission.m4          = 0.0f;
        break;

      case PHASE_RTL:
        shared.trip_rtl.armedAtMs            = activeArmedAt;
        shared.trip_rtl.launchLat            = activeLaunchLat;
        shared.trip_rtl.launchLon            = activeLaunchLon;
        shared.trip_rtl.state                = RTL_CLIMB;
        shared.cruise_rtl.targetAltFt        = RTL_ALTITUDE_FT;
        shared.cruise_rtl.yawTargetHeading   = currentHeading; // Keep current until climb finishes
        shared.cruise_rtl.targetRollDeg      = 0.0f;
        shared.cruise_rtl.targetPitchDeg     = 0.0f;
        shared.dashboard_rtl.m1              = 0.0f;
        shared.dashboard_rtl.m2              = 0.0f;
        shared.dashboard_rtl.m3              = 0.0f;
        shared.dashboard_rtl.m4              = 0.0f;
        break;

      case PHASE_HOVER_SETTLE:
        shared.trip_hoverSettle.enteredAtMs        = now;
        shared.cruise_hoverSettle.targetAltFt      = currentAlt;
        shared.cruise_hoverSettle.yawTargetHeading = currentHeading;
        shared.cruise_hoverSettle.targetRollDeg    = 0.0f;
        shared.cruise_hoverSettle.targetPitchDeg   = 0.0f;
        shared.dashboard_hoverSettle.m1            = 0.0f;
        shared.dashboard_hoverSettle.m2            = 0.0f;
        shared.dashboard_hoverSettle.m3            = 0.0f;
        shared.dashboard_hoverSettle.m4            = 0.0f;
        break;

      case PHASE_LANDING:
        shared.cruise_landing.targetAltFt      = currentAlt;
        shared.cruise_landing.yawTargetHeading = currentHeading;
        shared.cruise_landing.targetRollDeg    = 0.0f;
        shared.cruise_landing.targetPitchDeg   = 0.0f;
        shared.dashboard_landing.m1            = 0.0f;
        shared.dashboard_landing.m2            = 0.0f;
        shared.dashboard_landing.m3            = 0.0f;
        shared.dashboard_landing.m4            = 0.0f;
        break;

      case PHASE_LANDED: break;
    }
    shared.phase = next;
  });
}

// Safety wrapper to avoid repetitive checks
void checkCoreFailsafes(unsigned long armedAtMs, double launchLat, double launchLon) {
  bool tripRTL = false;
  
  // 1. Max Flight Time
  if (armedAtMs > 0 && (millis() - armedAtMs) >= MAX_FLIGHT_TIME_MS) {
    logLine("[SAFETY] Max flight time reached — forcing RTL.");
    tripRTL = true;
  }
  
  // 2. Geofence
  bool fix;
  double lat;
  double lon;
  unsigned long lastFix;
  
  withMutex([&]() { 
    fix     = shared.raw.gps.fix; 
    lat     = shared.raw.gps.lat; 
    lon     = shared.raw.gps.lon; 
    lastFix = shared.raw.gps.lastFixMs; 
  });
  
  if (fix && launchLat != 0.0) {
    if (gpsDistanceMeters(lat, lon, launchLat, launchLon) > GEOFENCE_RADIUS_M) {
      logLine("[SAFETY] Geofence exceeded — forcing RTL.");
      tripRTL = true;
    }
  }

  // 3. GPS Loss
  if (!fix && lastFix > 0 && (millis() - lastFix) >= GPS_LOSS_ABORT_MS) {
      logLine("[SAFETY] GPS fix lost — aborting directly to LANDING.");
      transitionTo(PHASE_LANDING); // Can't RTL without GPS
      return;
  }

  FlightPhase phase;
  withMutex([&]() { phase = shared.phase; });

  if (tripRTL && phase != PHASE_RTL && phase != PHASE_LANDING && phase != PHASE_LANDED) {
    transitionTo(PHASE_RTL);
  }
}

// ==========================================
// PER-PHASE NAV TICK HANDLERS (~10 Hz)
// ==========================================

void navTick_Parked(float /*navDt*/) {
  withMutex([&]() {
    shared.dashboard_parked.altitudeFt = shared.raw.baroAltitudeFt;
    shared.dashboard_parked.roll       = shared.raw.imu.gyroX; 
    shared.dashboard_parked.pitch      = shared.raw.imu.gyroY;
    shared.dashboard_parked.yaw        = shared.raw.imu.gyroZ;
  });
}

void navTick_Raise(float navDt) {
  withMutex([&]() {
    shared.dashboard_raise.altitudeFt     = shared.raw.baroAltitudeFt;
    shared.dashboard_raise.compassHeading = shared.raw.compassHeadingDeg;
    shared.dashboard_raise.roll           = shared.raw.imu.gyroX;
    shared.dashboard_raise.pitch          = shared.raw.imu.gyroY;
    shared.dashboard_raise.yaw            = shared.raw.imu.gyroZ;
  });

  bool ready = false;
  withMutex([&]() {
    float newTarget = shared.cruise_raise.targetAltFt + (RAISE_CLIMB_RATE_FPS * navDt);
    if (newTarget >= TAKEOFF_ALTITUDE_FT) {
      shared.cruise_raise.targetAltFt = TAKEOFF_ALTITUDE_FT;
      ready = true;
    } else {
      shared.cruise_raise.targetAltFt = newTarget;
    }
  });

  if (ready) {
    logLine("[NAV] Takeoff altitude reached, transitioning to HOLD.");
    transitionTo(PHASE_HOLD);
  }
}

void navTick_Hold(float /*navDt*/) {
  unsigned long armedAt;
  double launchLat;
  double launchLon;
  
  withMutex([&]() { 
    shared.dashboard_hold.altitudeFt     = shared.raw.baroAltitudeFt;
    shared.dashboard_hold.compassHeading = shared.raw.compassHeadingDeg;
    shared.dashboard_hold.roll           = shared.raw.imu.gyroX;
    shared.dashboard_hold.pitch          = shared.raw.imu.gyroY;
    shared.dashboard_hold.yaw            = shared.raw.imu.gyroZ;
    armedAt                              = shared.trip_hold.armedAtMs; 
    launchLat                            = shared.trip_hold.launchLat; 
    launchLon                            = shared.trip_hold.launchLon; 
  });
  
  checkCoreFailsafes(armedAt, launchLat, launchLon);
}

void navTick_Mission(float navDt) {
  RawGpsReading gps;
  Trip_Mission  trip;
  float         headingDeg;
  
  withMutex([&]() {
    gps        = shared.raw.gps; 
    trip       = shared.trip_mission; 
    headingDeg = shared.raw.compassHeadingDeg;
    
    shared.dashboard_mission.altitudeFt     = shared.raw.baroAltitudeFt;
    shared.dashboard_mission.compassHeading = headingDeg;
    shared.dashboard_mission.gpsLat         = gps.lat;
    shared.dashboard_mission.gpsLon         = gps.lon;
    shared.dashboard_mission.gpsFix         = gps.fix;
    shared.dashboard_mission.gpsSats        = gps.sats;
    shared.dashboard_mission.roll           = shared.raw.imu.gyroX;
    shared.dashboard_mission.pitch          = shared.raw.imu.gyroY;
    shared.dashboard_mission.yaw            = shared.raw.imu.gyroZ;
  });

  checkCoreFailsafes(trip.armedAtMs, trip.launchLat, trip.launchLon);
  withMutex([&]() { trip = shared.trip_mission; }); // Reload in case failsafe changed phase
  
  if (!trip.active) return;

  if (trip.currentWP >= trip.waypointCount) {
    transitionTo(PHASE_HOVER_SETTLE);
    logLine("[NAV] Mission complete — hovering before landing.");
    return;
  }

  Waypoint wp      = getMissionWaypoint(trip.currentWP, trip.launchLat, trip.launchLon);
  float    distM   = gpsDistanceMeters(gps.lat, gps.lon, wp.lat, wp.lon);
  float    bearing = gpsBearing(gps.lat, gps.lon, wp.lat, wp.lon);

  withMutex([&]() {
    shared.cruise_mission.yawTargetHeading = bearing;
    shared.dashboard_mission.distToWP      = distM;
    shared.dashboard_mission.bearingToWP   = bearing;
  });

  if (distM < WAYPOINT_ACCEPT_RADIUS_M) {
    logLine(String("[NAV] Reached waypoint ") + String(trip.currentWP) + " — advancing.");
    withMutex([&]() {
      shared.trip_mission.currentWP++;
      int next = shared.trip_mission.currentWP;
      shared.cruise_mission.targetAltFt = (next < shared.trip_mission.waypointCount)
          ? getMissionWaypoint(next, trip.launchLat, trip.launchLon).altFt : wp.altFt;
    });
    return;
  }

  float northM;
  float eastM;
  bearingToNorthEast(distM, bearing, northM, eastM);

  withMutex([&]() {
    shared.cruise_mission.targetRollDeg  = navEastPID.computeWithError(eastM,  navDt);
    shared.cruise_mission.targetPitchDeg = navNorthPID.computeWithError(northM, navDt);
  });
}

void navTick_RTL(float navDt) {
  RawGpsReading gps;
  Trip_RTL      trip;
  float         headingDeg;
  float         currentAlt;
  
  withMutex([&]() {
    gps        = shared.raw.gps; 
    trip       = shared.trip_rtl; 
    headingDeg = shared.raw.compassHeadingDeg; 
    currentAlt = shared.raw.baroAltitudeFt;
    
    shared.dashboard_rtl.altitudeFt     = currentAlt;
    shared.dashboard_rtl.compassHeading = headingDeg;
    shared.dashboard_rtl.roll           = shared.raw.imu.gyroX;
    shared.dashboard_rtl.pitch          = shared.raw.imu.gyroY;
    shared.dashboard_rtl.yaw            = shared.raw.imu.gyroZ;
  });

  if (trip.state == RTL_CLIMB) {
    if (currentAlt >= RTL_ALTITUDE_FT - 2.0f) {
      logLine("[RTL] Climb complete. Returning to launch.");
      withMutex([&]() { shared.trip_rtl.state = RTL_RETURN; });
    }
  } 
  else if (trip.state == RTL_RETURN) {
    float distM   = gpsDistanceMeters(gps.lat, gps.lon, trip.launchLat, trip.launchLon);
    float bearing = gpsBearing(gps.lat, gps.lon, trip.launchLat, trip.launchLon);
    
    if (distM < WAYPOINT_ACCEPT_RADIUS_M) {
      logLine("[RTL] Arrived over launch pad. Settling.");
      withMutex([&]() { 
        shared.trip_rtl.state            = RTL_SETTLE; 
        shared.trip_rtl.settleStartMs    = millis();
        shared.cruise_rtl.targetPitchDeg = 0.0f;
        shared.cruise_rtl.targetRollDeg  = 0.0f;
      });
      return;
    }

    float northM;
    float eastM;
    bearingToNorthEast(distM, bearing, northM, eastM);

    withMutex([&]() { shared.cruise_rtl.yawTargetHeading = bearing; });

    withMutex([&]() {
      shared.cruise_rtl.targetRollDeg  = navEastPID.computeWithError(eastM,  navDt);
      shared.cruise_rtl.targetPitchDeg = navNorthPID.computeWithError(northM, navDt);
    });
  } 
  else if (trip.state == RTL_SETTLE) {
    if (millis() - trip.settleStartMs >= 3000) {
      logLine("[RTL] Settle complete. Beginning landing.");
      transitionTo(PHASE_LANDING);
    }
  }
}

void navTick_HoverSettle(float /*navDt*/) {
  withMutex([&]() {
    shared.dashboard_hoverSettle.altitudeFt     = shared.raw.baroAltitudeFt;
    shared.dashboard_hoverSettle.compassHeading = shared.raw.compassHeadingDeg;
    shared.dashboard_hoverSettle.roll           = shared.raw.imu.gyroX;
    shared.dashboard_hoverSettle.pitch          = shared.raw.imu.gyroY;
    shared.dashboard_hoverSettle.yaw            = shared.raw.imu.gyroZ;
  });

  unsigned long enteredAt;
  withMutex([&]() { enteredAt = shared.trip_hoverSettle.enteredAtMs; });
  
  if (millis() - enteredAt >= MISSION_COMPLETE_HOVER_MS) {
    transitionTo(PHASE_LANDING);
    logLine("[NAV] Hover complete — beginning automatic landing.");
  }
}

void navTick_Landing(float navDt) {
  withMutex([&]() {
    shared.dashboard_landing.altitudeFt     = shared.raw.baroAltitudeFt;
    shared.dashboard_landing.compassHeading = shared.raw.compassHeadingDeg;
    shared.dashboard_landing.roll           = shared.raw.imu.gyroX;
    shared.dashboard_landing.pitch          = shared.raw.imu.gyroY;
    shared.dashboard_landing.yaw            = shared.raw.imu.gyroZ;
  });

  bool landed = false;
  withMutex([&]() {
    float newTarget = shared.cruise_landing.targetAltFt - (LAND_DESCENT_RATE_FPS * navDt);
    if (newTarget <= 0.0f) {
      shared.cruise_landing.targetAltFt = 0.0f;
      landed = true;
    } else {
      shared.cruise_landing.targetAltFt = newTarget;
    }
  });

  if (landed) {
    transitionTo(PHASE_LANDED);
    logLine("[NAV] Landed — motors disarmed.");
  }
}

void navTick_Landed(float /*navDt*/) {
  withMutex([&]() { shared.dashboard_landed.altitudeFt = shared.raw.baroAltitudeFt; });
}

// ==========================================
// PER-PHASE PHYSICS TICK HANDLERS (~200 Hz)
// ==========================================

void physicsTick_Parked(float dt) {
  (void)dt;
  altitudePID.reset(); 
  rollPID.reset(); 
  pitchPID.reset(); 
  yawPID.reset(); 
  navNorthPID.reset(); 
  navEastPID.reset();
}

void physicsTick_Raise(float dt) {
  Cruise_Raise c; 
  RawSensors   r;
  
  withMutex([&]() { 
    c = shared.cruise_raise; 
    r = shared.raw; 
  });

  MotorMix mix = computeMotorMix(
    c.targetAltFt, c.targetRollDeg, c.targetPitchDeg, c.yawTargetHeading,
    r.baroAltitudeFt, r.imu.gyroX, r.imu.gyroY, r.compassHeadingDeg, r.imu.gyroZ, dt);

  withMutex([&]() {
    shared.dashboard_raise.m1              = mix.m1; 
    shared.dashboard_raise.m2              = mix.m2;
    shared.dashboard_raise.m3              = mix.m3; 
    shared.dashboard_raise.m4              = mix.m4;
    shared.dashboard_raise.baseThrottle    = mix.baseThrottle;
    shared.dashboard_raise.rollCorrection  = mix.rollCorrection; 
    shared.dashboard_raise.pitchCorrection = mix.pitchCorrection;
  });
}

void physicsTick_Hold(float dt) {
  Cruise_Hold c; 
  RawSensors  r;
  
  withMutex([&]() { 
    c = shared.cruise_hold; 
    r = shared.raw; 
  });

  MotorMix mix = computeMotorMix(
    c.targetAltFt, c.targetRollDeg, c.targetPitchDeg, c.yawTargetHeading,
    r.baroAltitudeFt, r.imu.gyroX, r.imu.gyroY, r.compassHeadingDeg, r.imu.gyroZ, dt);

  withMutex([&]() {
    shared.dashboard_hold.m1              = mix.m1; 
    shared.dashboard_hold.m2              = mix.m2;
    shared.dashboard_hold.m3              = mix.m3; 
    shared.dashboard_hold.m4              = mix.m4;
    shared.dashboard_hold.baseThrottle    = mix.baseThrottle;
    shared.dashboard_hold.rollCorrection  = mix.rollCorrection; 
    shared.dashboard_hold.pitchCorrection = mix.pitchCorrection;
  });
}

void physicsTick_Mission(float dt) {
  Cruise_Mission c; 
  RawSensors     r;
  
  withMutex([&]() { 
    c = shared.cruise_mission; 
    r = shared.raw; 
  });

  MotorMix mix = computeMotorMix(
    c.targetAltFt, c.targetRollDeg, c.targetPitchDeg, c.yawTargetHeading,
    r.baroAltitudeFt, r.imu.gyroX, r.imu.gyroY, r.compassHeadingDeg, r.imu.gyroZ, dt);

  withMutex([&]() {
    shared.dashboard_mission.m1              = mix.m1; 
    shared.dashboard_mission.m2              = mix.m2;
    shared.dashboard_mission.m3              = mix.m3; 
    shared.dashboard_mission.m4              = mix.m4;
    shared.dashboard_mission.baseThrottle    = mix.baseThrottle;
    shared.dashboard_mission.rollCorrection  = mix.rollCorrection; 
    shared.dashboard_mission.pitchCorrection = mix.pitchCorrection;
  });
}

void physicsTick_RTL(float dt) {
  Cruise_RTL c; 
  RawSensors r;
  
  withMutex([&]() { 
    c = shared.cruise_rtl; 
    r = shared.raw; 
  });

  MotorMix mix = computeMotorMix(
    c.targetAltFt, c.targetRollDeg, c.targetPitchDeg, c.yawTargetHeading,
    r.baroAltitudeFt, r.imu.gyroX, r.imu.gyroY, r.compassHeadingDeg, r.imu.gyroZ, dt);

  withMutex([&]() {
    shared.dashboard_rtl.m1              = mix.m1; 
    shared.dashboard_rtl.m2              = mix.m2;
    shared.dashboard_rtl.m3              = mix.m3; 
    shared.dashboard_rtl.m4              = mix.m4;
    shared.dashboard_rtl.baseThrottle    = mix.baseThrottle;
    shared.dashboard_rtl.rollCorrection  = mix.rollCorrection; 
    shared.dashboard_rtl.pitchCorrection = mix.pitchCorrection;
  });
}

void physicsTick_HoverSettle(float dt) {
  Cruise_HoverSettle c; 
  RawSensors         r;
  
  withMutex([&]() { 
    c = shared.cruise_hoverSettle; 
    r = shared.raw; 
  });

  MotorMix mix = computeMotorMix(
    c.targetAltFt, c.targetRollDeg, c.targetPitchDeg, c.yawTargetHeading,
    r.baroAltitudeFt, r.imu.gyroX, r.imu.gyroY, r.compassHeadingDeg, r.imu.gyroZ, dt);

  withMutex([&]() {
    shared.dashboard_hoverSettle.m1              = mix.m1; 
    shared.dashboard_hoverSettle.m2              = mix.m2;
    shared.dashboard_hoverSettle.m3              = mix.m3; 
    shared.dashboard_hoverSettle.m4              = mix.m4;
    shared.dashboard_hoverSettle.baseThrottle    = mix.baseThrottle;
    shared.dashboard_hoverSettle.rollCorrection  = mix.rollCorrection; 
    shared.dashboard_hoverSettle.pitchCorrection = mix.pitchCorrection;
  });
}

void physicsTick_Landing(float dt) {
  Cruise_Landing c; 
  RawSensors     r;
  
  withMutex([&]() { 
    c = shared.cruise_landing; 
    r = shared.raw; 
  });

  MotorMix mix = computeMotorMix(
    c.targetAltFt, c.targetRollDeg, c.targetPitchDeg, c.yawTargetHeading,
    r.baroAltitudeFt, r.imu.gyroX, r.imu.gyroY, r.compassHeadingDeg, r.imu.gyroZ, dt);

  withMutex([&]() {
    shared.dashboard_landing.m1              = mix.m1; 
    shared.dashboard_landing.m2              = mix.m2;
    shared.dashboard_landing.m3              = mix.m3; 
    shared.dashboard_landing.m4              = mix.m4;
    shared.dashboard_landing.baseThrottle    = mix.baseThrottle;
    shared.dashboard_landing.rollCorrection  = mix.rollCorrection; 
    shared.dashboard_landing.pitchCorrection = mix.pitchCorrection;
  });
}

void physicsTick_Landed(float dt) {
  (void)dt;
  altitudePID.reset(); 
  rollPID.reset(); 
  pitchPID.reset(); 
  yawPID.reset(); 
  navNorthPID.reset(); 
  navEastPID.reset();
}

// ==========================================
// TASKS
// ==========================================
void navigationTask(void* parameter) {
  const TickType_t xFrequency = pdMS_TO_TICKS(NAV_LOOP_MS);
  TickType_t lastWakeTime     = xTaskGetTickCount();
  const float navDt           = NAV_LOOP_MS / 1000.0f;

  for (;;) {
#ifdef WOKWI_SIM
    withMutex([&]() {
      if (shared.raw.gps.fix) shared.raw.gps.lastFixMs = millis();
      shared.raw.gps.sats = shared.raw.gps.fix ? 8 : 0;
    });
#else
    while (Serial2.available() > 0) gps.encode(Serial2.read());
    if (gps.location.isValid() && gps.location.age() < 2000) {
      RawGpsReading g;
      g.lat      = gps.location.lat(); 
      g.lon      = gps.location.lng();
      g.fix      = true; 
      g.sats     = gps.satellites.value(); 
      g.speedMps = gps.speed.mps(); 
      g.lastFixMs = millis();
      withMutex([&]() { shared.raw.gps = g; });
    } else {
      withMutex([&]() { shared.raw.gps.fix = false; });
    }
    compass.read();
    float heading = compass.getAzimuth();
    withMutex([&]() { shared.raw.compassHeadingDeg = heading; });
#endif

    FlightPhase phase;
    withMutex([&]() { phase = shared.phase; });

    switch (phase) {
      case PHASE_PARKED:       navTick_Parked(navDt);      break;
      case PHASE_RAISE:        navTick_Raise(navDt);       break;
      case PHASE_HOLD:         navTick_Hold(navDt);        break;
      case PHASE_MISSION:      navTick_Mission(navDt);     break;
      case PHASE_RTL:          navTick_RTL(navDt);         break;
      case PHASE_HOVER_SETTLE: navTick_HoverSettle(navDt); break;
      case PHASE_LANDING:      navTick_Landing(navDt);     break;
      case PHASE_LANDED:       navTick_Landed(navDt);      break;
      default: PANIC("navigationTask: unhandled FlightPhase");
    }
    vTaskDelayUntil(&lastWakeTime, xFrequency);
  }
}

unsigned long lastGyroMicros = 0;
void physicsTask(void* parameter) {
  lastGyroMicros = micros();
  filter.begin(PHYSICS_LOOP_HZ);
  const TickType_t xFrequency = pdMS_TO_TICKS(PHYSICS_LOOP_MS);
  TickType_t lastWakeTime     = xTaskGetTickCount();

  for (;;) {
    unsigned long now = micros();
    float dt          = (now - lastGyroMicros) / 1000000.0f;
    lastGyroMicros    = now;

    sensors_event_t a;
    sensors_event_t g;
    sensors_event_t temp;
    mpu.getEvent(&a, &g, &temp);

    float gx = g.gyro.x * 57.2958f;
    float gy = g.gyro.y * 57.2958f;
    float gz = g.gyro.z * 57.2958f;

    if (dt > 0 && dt < 1.0f) {
      filter.updateIMU(gx, gy, gz, a.acceleration.x, a.acceleration.y, a.acceleration.z);
    }

    float baroAlt = ((float)barometer.readAltitudeMeters() * 3.28084f) - groundAltitudeFt;

    withMutex([&]() {
      shared.raw.imu.gyroX = filter.getRoll(); 
      shared.raw.imu.gyroY = filter.getPitch(); 
      shared.raw.imu.gyroZ = filter.getYaw();
      shared.raw.imu.accX  = a.acceleration.x; 
      shared.raw.imu.accY  = a.acceleration.y; 
      shared.raw.imu.accZ  = a.acceleration.z;
      shared.raw.imu.temp  = temp.temperature; 
      shared.raw.baroAltitudeFt = baroAlt;
    });

    FlightPhase phase;
    withMutex([&]() { phase = shared.phase; });

    switch (phase) {
      case PHASE_PARKED:       physicsTick_Parked(dt);      break;
      case PHASE_RAISE:        physicsTick_Raise(dt);       break;
      case PHASE_HOLD:         physicsTick_Hold(dt);        break;
      case PHASE_MISSION:      physicsTick_Mission(dt);     break;
      case PHASE_RTL:          physicsTick_RTL(dt);         break;
      case PHASE_HOVER_SETTLE: physicsTick_HoverSettle(dt); break;
      case PHASE_LANDING:      physicsTick_Landing(dt);     break;
      case PHASE_LANDED:       physicsTick_Landed(dt);      break;
      default: PANIC("physicsTask: unhandled FlightPhase");
    }
    vTaskDelayUntil(&lastWakeTime, xFrequency);
  }
}

// ==========================================
// WEB FORMATTERS
// ==========================================
String getGyroReadings() {
  float roll;
  float pitch;
  float yaw;
  withMutex([&]() { 
    roll  = shared.raw.imu.gyroX; 
    pitch = shared.raw.imu.gyroY; 
    yaw   = shared.raw.imu.gyroZ; 
  });

  JsonDocument doc;
  doc["gyroX"] = roll; 
  doc["gyroY"] = pitch; 
  doc["gyroZ"] = yaw;
  
  String out; 
  serializeJson(doc, out); 
  return out;
}

String getAccReadings() {
  float ax;
  float ay;
  float az;
  withMutex([&]() { 
    ax = shared.raw.imu.accX; 
    ay = shared.raw.imu.accY; 
    az = shared.raw.imu.accZ; 
  });

  JsonDocument doc;
  doc["accX"] = ax; 
  doc["accY"] = ay; 
  doc["accZ"] = az;
  
  String out; 
  serializeJson(doc, out); 
  return out;
}

String getFlightReadings() {
  FlightPhase phase;
  float baroAlt;
  withMutex([&]() { 
    phase   = shared.phase; 
    baroAlt = shared.raw.baroAltitudeFt;
  });

  JsonDocument readings;
  readings["flightPhase"]   = phaseName(phase);
  readings["flightEnabled"] = phaseFlightEnabled(phase);
  readings["altFt"]         = baroAlt;

  switch (phase) {
    case PHASE_PARKED:
    case PHASE_LANDED: {
      withMutex([&]() {
        readings["targetFt"]           = 0.0f;
        readings["m1"]                 = 0; 
        readings["m2"]                 = 0; 
        readings["m3"]                 = 0; 
        readings["m4"]                 = 0;
        readings["baseThrottle"]       = 0; 
        readings["rollCorrection"]     = 0; 
        readings["pitchCorrection"]    = 0;
        readings["gpsFix"]             = shared.raw.gps.fix; 
        readings["gpsLat"]             = shared.raw.gps.lat; 
        readings["gpsLon"]             = shared.raw.gps.lon; 
        readings["gpsSats"]            = shared.raw.gps.sats;
        readings["compassHeading"]     = shared.raw.compassHeadingDeg;
        readings["navActive"]          = false; 
        readings["navWaypoint"]        = 0; 
        readings["navWaypointCount"]   = 0; 
        readings["navDistM"]           = 0.0; 
        readings["navBearing"]         = 0.0;
        readings["flightSecRemaining"] = (long)(MAX_FLIGHT_TIME_MS / 1000);
      });
      break;
    }
    case PHASE_RAISE: {
      Cruise_Raise    c; 
      Dashboard_Raise db;
      float heading;
      withMutex([&]() { 
        c       = shared.cruise_raise; 
        db      = shared.dashboard_raise;
        heading = shared.raw.compassHeadingDeg;
      });
      
      readings["targetFt"]        = c.targetAltFt; 
      readings["m1"]              = (int)(db.m1 * 100); 
      readings["m2"]              = (int)(db.m2 * 100); 
      readings["m3"]              = (int)(db.m3 * 100); 
      readings["m4"]              = (int)(db.m4 * 100);
      readings["baseThrottle"]    = (int)(db.baseThrottle * 100); 
      readings["rollCorrection"]  = (int)(db.rollCorrection * 100); 
      readings["pitchCorrection"] = (int)(db.pitchCorrection * 100);
      readings["compassHeading"]  = heading; 
      readings["navActive"]       = false;
      break;
    }
    case PHASE_HOLD: {
      Cruise_Hold   c; 
      Dashboard_Hold db; 
      Trip_Hold     t; 
      RawGpsReading g;
      float heading;
      withMutex([&]() { 
        c       = shared.cruise_hold; 
        db      = shared.dashboard_hold; 
        t       = shared.trip_hold; 
        g       = shared.raw.gps;
        heading = shared.raw.compassHeadingDeg;
      });
      
      readings["targetFt"]           = c.targetAltFt; 
      readings["m1"]                 = (int)(db.m1 * 100); 
      readings["m2"]                 = (int)(db.m2 * 100); 
      readings["m3"]                 = (int)(db.m3 * 100); 
      readings["m4"]                 = (int)(db.m4 * 100);
      readings["baseThrottle"]       = (int)(db.baseThrottle * 100); 
      readings["rollCorrection"]     = (int)(db.rollCorrection * 100); 
      readings["pitchCorrection"]    = (int)(db.pitchCorrection * 100);
      readings["compassHeading"]     = heading; 
      readings["gpsFix"]             = g.fix; 
      readings["gpsLat"]             = g.lat; 
      readings["gpsLon"]             = g.lon; 
      readings["gpsSats"]            = g.sats;
      readings["flightSecRemaining"] = t.armedAtMs > 0 ? max(0L, (long)((MAX_FLIGHT_TIME_MS - (millis() - t.armedAtMs)) / 1000)) : (long)(MAX_FLIGHT_TIME_MS / 1000);
      break;
    }
    case PHASE_MISSION: {
      Dashboard_Mission d; 
      Cruise_Mission    c; 
      Trip_Mission      t;
      withMutex([&]() { 
        d = shared.dashboard_mission; 
        c = shared.cruise_mission; 
        t = shared.trip_mission; 
      });
      
      readings["targetFt"]           = c.targetAltFt; 
      readings["m1"]                 = (int)(d.m1 * 100); 
      readings["m2"]                 = (int)(d.m2 * 100); 
      readings["m3"]                 = (int)(d.m3 * 100); 
      readings["m4"]                 = (int)(d.m4 * 100);
      readings["baseThrottle"]       = (int)(d.baseThrottle * 100); 
      readings["rollCorrection"]     = (int)(d.rollCorrection * 100); 
      readings["pitchCorrection"]    = (int)(d.pitchCorrection * 100);
      readings["compassHeading"]     = d.compassHeading; 
      readings["gpsFix"]             = d.gpsFix; 
      readings["gpsLat"]             = d.gpsLat; 
      readings["gpsLon"]             = d.gpsLon; 
      readings["gpsSats"]            = d.gpsSats;
      readings["navActive"]          = t.active; 
      readings["navWaypoint"]        = t.currentWP; 
      readings["navWaypointCount"]   = t.waypointCount; 
      readings["navDistM"]           = d.distToWP; 
      readings["navBearing"]         = d.bearingToWP;
      readings["flightSecRemaining"] = t.armedAtMs > 0 ? max(0L, (long)((MAX_FLIGHT_TIME_MS - (millis() - t.armedAtMs)) / 1000)) : (long)(MAX_FLIGHT_TIME_MS / 1000);
      break;
    }
    case PHASE_RTL: {
      Dashboard_RTL d; 
      Cruise_RTL    c; 
      Trip_RTL      t;
      withMutex([&]() { 
        d = shared.dashboard_rtl; 
        c = shared.cruise_rtl; 
        t = shared.trip_rtl; 
      });
      
      readings["targetFt"]           = c.targetAltFt; 
      readings["m1"]                 = (int)(d.m1 * 100); 
      readings["m2"]                 = (int)(d.m2 * 100); 
      readings["m3"]                 = (int)(d.m3 * 100); 
      readings["m4"]                 = (int)(d.m4 * 100);
      readings["baseThrottle"]       = (int)(d.baseThrottle * 100); 
      readings["rollCorrection"]     = (int)(d.rollCorrection * 100); 
      readings["pitchCorrection"]    = (int)(d.pitchCorrection * 100);
      readings["compassHeading"]     = d.compassHeading; 
      readings["navActive"]          = true; 
      readings["flightSecRemaining"] = t.armedAtMs > 0 ? max(0L, (long)((MAX_FLIGHT_TIME_MS - (millis() - t.armedAtMs)) / 1000)) : (long)(MAX_FLIGHT_TIME_MS / 1000);
      break;
    }
    case PHASE_HOVER_SETTLE: {
      Cruise_HoverSettle    c; 
      Dashboard_HoverSettle db;
      float heading;
      withMutex([&]() { 
        c       = shared.cruise_hoverSettle; 
        db      = shared.dashboard_hoverSettle;
        heading = shared.raw.compassHeadingDeg;
      });
      
      readings["targetFt"]       = c.targetAltFt; 
      readings["m1"]             = (int)(db.m1 * 100); 
      readings["m2"]             = (int)(db.m2 * 100); 
      readings["m3"]             = (int)(db.m3 * 100); 
      readings["m4"]             = (int)(db.m4 * 100);
      readings["compassHeading"] = heading; 
      readings["navActive"]      = false;
      break;
    }
    case PHASE_LANDING: {
      Cruise_Landing    c; 
      Dashboard_Landing db;
      float heading;
      withMutex([&]() { 
        c       = shared.cruise_landing; 
        db      = shared.dashboard_landing;
        heading = shared.raw.compassHeadingDeg;
      });
      
      readings["targetFt"]       = c.targetAltFt; 
      readings["m1"]             = (int)(db.m1 * 100); 
      readings["m2"]             = (int)(db.m2 * 100); 
      readings["m3"]             = (int)(db.m3 * 100); 
      readings["m4"]             = (int)(db.m4 * 100);
      readings["compassHeading"] = heading; 
      readings["navActive"]      = false;
      break;
    }
    default: PANIC("getFlightReadings: unhandled FlightPhase");
  }

  String out; 
  serializeJson(readings, out); 
  return out;
}

// ==========================================
// INIT HELPERS
// ==========================================
void initMPU() {
  logLine("[IMU] Initializing MPU6050...");
  if (!mpu.begin()) { 
    logLine("[IMU] ERROR: MPU6050 not found."); 
    while (1) { delay(10); } 
  }
  logLine("[IMU] MPU6050 ready.");
}

void loadCompassCalibration() {
  compassPrefs.begin("compass", true);
  bool hasCal = compassPrefs.isKey("offX");
  if (hasCal) {
    compassOffsetX = compassPrefs.getFloat("offX", 0); 
    compassOffsetY = compassPrefs.getFloat("offY", 0); 
    compassOffsetZ = compassPrefs.getFloat("offZ", 0);
    compassScaleX  = compassPrefs.getFloat("sclX", 1); 
    compassScaleY  = compassPrefs.getFloat("sclY", 1); 
    compassScaleZ  = compassPrefs.getFloat("sclZ", 1);
  }
  compassPrefs.end();
  compass.setCalibration(compassOffsetX, compassOffsetY, compassOffsetZ, compassScaleX,  compassScaleY,  compassScaleZ);
}

void runCompassCalibration() {
  logLine("[COMPASS] Calibration starting — rotate drone slowly through all axes now...");
  int16_t minX = 32767;
  int16_t maxX = -32768;
  int16_t minY = 32767;
  int16_t maxY = -32768;
  int16_t minZ = 32767;
  int16_t maxZ = -32768;
  
  unsigned long start = millis();
  while (millis() - start < COMPASS_CAL_DURATION_MS) {
    compass.read();
    int16_t x = compass.getX();
    int16_t y = compass.getY();
    int16_t z = compass.getZ();
    
    minX = min(minX, x); 
    maxX = max(maxX, x); 
    minY = min(minY, y); 
    maxY = max(maxY, y); 
    minZ = min(minZ, z); 
    maxZ = max(maxZ, z);
    
    delay(50);
  }
  
  compassOffsetX = (minX + maxX) / 2.0f; 
  compassOffsetY = (minY + maxY) / 2.0f; 
  compassOffsetZ = (minZ + maxZ) / 2.0f;
  
  float rangeX = (maxX - minX) / 2.0f;
  float rangeY = (maxY - minY) / 2.0f;
  float rangeZ = (maxZ - minZ) / 2.0f;
  float avg    = (rangeX + rangeY + rangeZ) / 3.0f;
  
  compassScaleX = (rangeX > 0) ? (avg / rangeX) : 1.0f; 
  compassScaleY = (rangeY > 0) ? (avg / rangeY) : 1.0f; 
  compassScaleZ = (rangeZ > 0) ? (avg / rangeZ) : 1.0f;
  
  compass.setCalibration(compassOffsetX, compassOffsetY, compassOffsetZ, compassScaleX, compassScaleY, compassScaleZ);
  
  compassPrefs.begin("compass", false);
  compassPrefs.putFloat("offX", compassOffsetX); 
  compassPrefs.putFloat("offY", compassOffsetY); 
  compassPrefs.putFloat("offZ", compassOffsetZ);
  compassPrefs.putFloat("sclX", compassScaleX); 
  compassPrefs.putFloat("sclY", compassScaleY); 
  compassPrefs.putFloat("sclZ", compassScaleZ);
  compassPrefs.end();
  
  logLine("[COMPASS] Calibration saved.");
}

void initCompass() {
  logLine("[COMPASS] Initializing QMC5883L...");
  compass.init(); 
  compass.setMode(0x01, 0x0C, 0x10, 0xC0);
  
  if (CALIBRATE_COMPASS_ON_BOOT) {
    runCompassCalibration(); 
  } else {
    loadCompassCalibration();
  }
  
  logLine("[COMPASS] QMC5883L ready.");
}

void initGPS() { 
  Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN); 
}

// ESC calibration + arming runs every boot (props OFF, battery on):
//   1. MAX throttle pulse -> ESC beeps high (recognises top of range)
//   2. MIN throttle pulse -> ESC beeps low (saves range to EEPROM)
//   3. ARM pulse held     -> ESC ready to accept throttle commands
// Total ~8 s. Safe to repeat every boot; ESCs re-save the same values.
// Adjust EspESC:: calls below to match your EspESC.hpp API.
void initESC() {
  EspESC::begin();

  logLine("[ESC] Calibrating -- REMOVE PROPS. Sending MAX throttle for 3 s...");
  EspESC::writeAllMicroseconds(ESC_PULSE_MAX_US);
  delay(3000);

  logLine("[ESC] Sending MIN throttle -- ESC should beep to confirm range saved.");
  EspESC::writeAllMicroseconds(ESC_PULSE_MIN_US);
  delay(3000);

  logLine("[ESC] Arming -- holding min throttle for 2 s...");
  EspESC::writeAllMicroseconds(ESC_PULSE_ARM_US);
  delay(2000);

  logLine("[ESC] ESCs calibrated and armed.");
}

void initLittleFS() {
  if (!LittleFS.begin(false, "/littlefs", 10, "spiffs")) {
    logLine("[FS] LittleFS mount failed — formatting and retrying...");
    LittleFS.format();
    if (!LittleFS.begin(false, "/littlefs", 10, "spiffs")) {
      PANIC("LittleFS failed to mount after format");
    }
  }
}

void initWiFi() {
  WiFi.mode(WIFI_STA); 
  WiFi.begin(ssid, password);
  logLine("[WIFI] Connecting...");
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - wifiStart > 15000) {
      PANIC("WiFi failed to connect within 15 s — check SSID/password");
    }
    delay(500);
  }
  logLine(String("[WIFI] Connected, IP: ") + WiFi.localIP().toString());
}

// ==========================================
// SIMULATION
// ==========================================
#ifdef WOKWI_SIM
void parseSimInput() {
  static String buf = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      if (buf.startsWith("HDG:")) { 
        withMutex([&]() { shared.raw.compassHeadingDeg = buf.substring(4).toFloat(); }); 
      }
      else if (buf.startsWith("LAT:")) { 
        withMutex([&]() { shared.raw.gps.lat = buf.substring(4).toDouble(); }); 
      }
      else if (buf.startsWith("LON:")) { 
        withMutex([&]() { shared.raw.gps.lon = buf.substring(4).toDouble(); }); 
      }
      else if (buf.startsWith("FIX:")) { 
        withMutex([&]() { shared.raw.gps.fix = (buf.substring(4).toInt() == 1); }); 
      }
      else if (buf.startsWith("MISSION:")) {
        bool fixNow; 
        withMutex([&]() { fixNow = shared.raw.gps.fix; });
        if (fixNow) transitionTo(PHASE_MISSION);
      }
      buf = "";
    } else if (c != '\r') { 
      buf += c; 
    }
  }
}
#endif

// ==========================================
// CRSF RC TASK -- START / STOP ONLY
// ==========================================
// How CRSF packets work:
//   Every ~4 ms the ELRS receiver sends a 26-byte frame at 420000 baud.
//   Byte 0:    sync (0xC8)
//   Byte 1:    payload length (24)
//   Byte 2:    frame type (0x16 = RC channels packed)
//   Bytes 3-24: 16 channels packed as 11-bit values (176 bits)
//   Byte 25:   CRC8
//   Raw channel values: 172 (min) to 1811 (max), midpoint 992.
//
// Helper: extract one 11-bit channel value from the packed payload.
static uint16_t crsfChannel(const uint8_t* payload, int chIdx) {
  int      bitOffset = chIdx * 11;
  int      byteIdx   = bitOffset / 8;
  int      bitIdx    = bitOffset % 8;
  uint32_t raw = ((uint32_t)payload[byteIdx])
               | ((uint32_t)payload[byteIdx + 1] << 8)
               | ((uint32_t)payload[byteIdx + 2] << 16);
  return (raw >> bitIdx) & 0x7FF;
}

void crsfTask(void* parameter) {
  Serial1.begin(CRSF_BAUD, SERIAL_8N1, CRSF_RX_PIN, -1 /* TX unused */);
  logLine("[CRSF] Listening -- Ch5=START, Ch6=STOP");

  uint8_t buf[64];
  int     bufLen       = 0;
  bool    prevStartHigh = false; // for edge detection on START channel

  for (;;) {
    while (Serial1.available()) {
      uint8_t b = Serial1.read();

      // Wait for CRSF sync byte before starting a frame
      if (bufLen == 0 && b != 0xC8) continue;
      buf[bufLen++] = b;

      if (bufLen < 3) continue;

      int frameLen = buf[1] + 2; // payload length + 2 header bytes

      // Overflow guard: if we somehow accumulated garbage, reset
      if (bufLen > frameLen || bufLen >= (int)sizeof(buf)) {
        bufLen = 0;
        continue;
      }

      if (bufLen < frameLen) continue; // frame not complete yet

      // We have a full frame -- process it
      if (buf[2] == 0x16 && frameLen == 26) {
        // payload starts at byte 3
        const uint8_t* payload = buf + 3;

        uint16_t startVal = crsfChannel(payload, CRSF_START_CH);
        uint16_t stopVal  = crsfChannel(payload, CRSF_STOP_CH);

        // -- STOP: level-triggered, highest priority --
        // Any frame with Ch6 low cuts motors immediately, regardless of phase.
        if (stopVal < CRSF_LOW_THRESHOLD) {
          FlightPhase phase;
          withMutex([&]() { phase = shared.phase; });
          if (phase != PHASE_PARKED && phase != PHASE_LANDED) {
            logLine("[CRSF] STOP switch -- emergency stop.");
            transitionTo(PHASE_PARKED);
          }
        }

        // -- START: edge-triggered (LOW->HIGH transition only) --
        // Only triggers action when Ch5 crosses UP, so holding the switch does nothing extra.
        bool startHigh = (startVal > CRSF_HIGH_THRESHOLD);
        if (startHigh && !prevStartHigh) {
          FlightPhase phase;
          bool hasFix;
          withMutex([&]() { phase = shared.phase; hasFix = shared.raw.gps.fix; });
          
          if (phase == PHASE_PARKED) {
            if (!hasFix) {
              logLine("[CRSF] START ignored -- no GPS fix.");
            } else {
              logLine("[CRSF] START switch -- arming and taking off.");
              transitionTo(PHASE_RAISE);
            }
          } else if (phase == PHASE_HOLD) {
             logLine("[CRSF] START switch -- starting waypoint mission.");
             transitionTo(PHASE_MISSION);
          } else {
            logLine("[CRSF] START ignored -- already flying/busy (phase: "
                    + String(phaseName(phase)) + ")");
          }
        }
        prevStartHigh = startHigh;
      }

      bufLen = 0; // done with this frame, reset for next
    }
    vTaskDelay(pdMS_TO_TICKS(2)); // yield; 2 ms is well within the 4 ms frame interval
  }
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  sharedDataMutex = xSemaphoreCreateMutex(); 
  serialMutex     = xSemaphoreCreateMutex();
  
  Serial.begin(115200); 
  Wire.begin(16, 15);
  
  initWiFi(); 
  initLittleFS(); 
  initMPU();
  
#ifndef WOKWI_SIM
  initCompass(); 
  initGPS();
  initESC();
#endif
  barometer.initialize();
  
  // Average 20 barometer readings over ~2 s so the sensor has time to settle
  // and temperature effects are smoothed before we lock in the ground reference.
  logLine("[BARO] Sampling ground altitude (20 readings)...");
  {
    const int   BARO_SAMPLES    = 20;
    const int   BARO_INTERVAL_MS = 100;
    float accum = 0.0f;
    for (int i = 0; i < BARO_SAMPLES; i++) {
      accum += (float)barometer.readAltitudeMeters() * 3.28084f;
      delay(BARO_INTERVAL_MS);
    }
    groundAltitudeFt = accum / BARO_SAMPLES;
  }
  logLine(String("[BARO] Ground altitude locked: ") + String(groundAltitudeFt, 1) + " ft");

  xTaskCreatePinnedToCore(navigationTask, "NavTask",    8192, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(physicsTask,    "PhysicsTask", 8192, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(crsfTask,       "CRSFTask",    4096, NULL, 1, NULL, 0);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* r) { 
    r->send(LittleFS, "/index.html", "text/html"); 
  });
  
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  server.on("/takeoff", HTTP_GET, [](AsyncWebServerRequest* r) {
    bool hasFix; 
    withMutex([&]() { hasFix = shared.raw.gps.fix; });
    if (!hasFix) { 
      r->send(400, "text/plain", "NO GPS FIX — lock needed before arming"); 
      return; 
    }
    transitionTo(PHASE_RAISE);
    logLine("[FLIGHT] Takeoff requested. Drone Armed.");
    r->send(200, "text/plain", "TAKEOFF INITIATED");
  });

  server.on("/start-waypoint-nav", HTTP_GET, [](AsyncWebServerRequest* r) {
    FlightPhase currentPhase; 
    withMutex([&]() { currentPhase = shared.phase; });
    if (currentPhase != PHASE_HOLD) { 
      r->send(400, "text/plain", "Drone must be in HOLD to start mission"); 
      return; 
    }
    transitionTo(PHASE_MISSION);
    logLine("[FLIGHT] Mission Started.");
    r->send(200, "text/plain", "WAYPOINT NAVIGATION STARTED");
  });

  server.on("/rtl", HTTP_GET, [](AsyncWebServerRequest* r) {
    FlightPhase currentPhase;
    withMutex([&]() { currentPhase = shared.phase; });
    if (currentPhase == PHASE_PARKED || currentPhase == PHASE_LANDED) {
      r->send(400, "text/plain", "Cannot RTL — drone is not airborne");
      return;
    }
    transitionTo(PHASE_RTL);
    logLine("[FLIGHT] Return to Launch triggered.");
    r->send(200, "text/plain", "RTL INITIATED");
  });

  server.on("/abort", HTTP_GET, [](AsyncWebServerRequest* r) {
    FlightPhase currentPhase; 
    withMutex([&]() { currentPhase = shared.phase; });
    if (currentPhase == PHASE_MISSION || currentPhase == PHASE_RTL) {
      transitionTo(PHASE_HOLD);
    }
    logLine("[MISSION] Aborted — holding position.");
    r->send(200, "text/plain", "ABORTED — HOLDING");
  });

  server.on("/land", HTTP_GET, [](AsyncWebServerRequest* r) {
    transitionTo(PHASE_LANDING);
    logLine("[FLIGHT] Manual landing requested.");
    r->send(200, "text/plain", "LANDING");
  });

  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest* r) {
    transitionTo(PHASE_PARKED);
    logLine("[FLIGHT] EMERGENCY STOP — motors cut.");
    r->send(200, "text/plain", "STOPPED");
  });

  // Motor test endpoint — PROPS OFF, drone must be PARKED.
  // Usage: GET /motor-test?motor=1&pct=15
  //   motor: 1-4 (matches M1-M4 layout in your frame)
  //   pct:   0-30 (throttle percentage — capped at 30% for bench safety)
  // The motor spins for 2 seconds then stops automatically.
  // Use this to verify spin direction and motor-to-ESC wiring before first flight.
  server.on("/motor-test", HTTP_GET, [](AsyncWebServerRequest* r) {
    FlightPhase currentPhase;
    withMutex([&]() { currentPhase = shared.phase; });
    if (currentPhase != PHASE_PARKED) {
      r->send(400, "text/plain", "Motor test only allowed while PARKED");
      return;
    }
    if (!r->hasParam("motor") || !r->hasParam("pct")) {
      r->send(400, "text/plain", "Required params: motor=1-4 & pct=0-30");
      return;
    }

    int motor = r->getParam("motor")->value().toInt();
    int pct   = r->getParam("pct")->value().toInt();

    if (motor < 1 || motor > 4) {
      r->send(400, "text/plain", "motor must be 1-4");
      return;
    }
    // Hard cap at 30% — enough to confirm spin, not enough to lift off.
    pct = constrain(pct, 0, 30);
    float throttle = pct / 100.0f;

    logLine(String("[MOTOR TEST] M") + motor + " at " + pct + "% for 2 s — PROPS OFF?");

    // Spin the requested motor for 2 s then cut.
    // EspESC::writeMotor(index, 0.0-1.0) assumed — adjust to your EspESC API.
    EspESC::writeMotor(motor - 1, throttle); // 0-indexed
    delay(2000);
    EspESC::writeMotor(motor - 1, 0.0f);

    logLine(String("[MOTOR TEST] M") + motor + " stopped.");
    r->send(200, "text/plain",
      String("M") + motor + " ran at " + pct + "% for 2 s — check spin direction in log");
  });

  server.begin();
}

// ==========================================
// LOOP (Core 0 — SSE dispatch only)
// ==========================================
unsigned long lastGyroSend   = 0;
unsigned long lastAccSend    = 0;
unsigned long lastFlightSend = 0;

void loop() {
#ifdef WOKWI_SIM
  parseSimInput();
#endif
}