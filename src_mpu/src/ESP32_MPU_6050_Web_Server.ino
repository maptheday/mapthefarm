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
const unsigned long PHYSICS_LOOP_MS  = 5;
const float         PHYSICS_LOOP_HZ  = 1000.0f / PHYSICS_LOOP_MS;
const unsigned long SSE_GYRO_MS      = 10;
const unsigned long SSE_ACC_MS       = 200;
const unsigned long SSE_FLIGHT_MS    = 100;
const unsigned long NAV_LOOP_MS      = 100;

// ==========================================
// SAFETY CONFIGURATION
// ==========================================
const unsigned long MAX_FLIGHT_TIME_MS  = 5UL * 60UL * 1000UL;
const float         GEOFENCE_RADIUS_M   = 150.0f;
const unsigned long GPS_LOSS_ABORT_MS   = 3000;

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
float compassOffsetX = 0, compassOffsetY = 0, compassOffsetZ = 0;
float compassScaleX  = 1, compassScaleY  = 1, compassScaleZ  = 1;

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
const int   WAYPOINT_COUNT              = sizeof(WAYPOINTS) / sizeof(WAYPOINTS[0]);
const float WAYPOINT_ACCEPT_RADIUS_M    = 5.0f;
const unsigned long MISSION_COMPLETE_HOVER_MS = 10000;
const float LAND_DESCENT_RATE_FPS       = 1.5f;

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
AsyncEventSource  events("/events");

// ==========================================
// PID CONTROLLERS
// ==========================================
PID altitudePID (0.08f,   0.01f,   0.05f,  0.0f,   1.0f  );
PID rollPID     (0.01f,   0.001f,  0.005f, -0.3f,  0.3f  );
PID pitchPID    (0.01f,   0.001f,  0.005f, -0.3f,  0.3f  );
PID yawPID      (0.005f,  0.0001f, 0.001f, -0.2f,  0.2f  );
PID navNorthPID (0.5f,    0.0f,    0.1f,   -15.0f, 15.0f );
PID navEastPID  (0.5f,    0.0f,    0.1f,   -15.0f, 15.0f );

// ==========================================
// PANIC — embedded equivalent of an unhandled exception.
// Prints to Serial and halts. physicsTask stalls → motors stop.
// ==========================================
#define PANIC(msg) do { Serial.println(F("[PANIC] " msg " — halting")); while(1) { delay(10); } } while(0)

// Guard for any control-path code that reads dashboard motor outputs.
// Zero across all four motors is physically impossible on an armed drone
// (gravity demands throttle), so it reliably means the physics task hasn't
// written this phase's dashboard yet.
#define ASSERT_MOTORS_INITIALIZED(d) \
  do { if ((d).m1 == 0.0f && (d).m2 == 0.0f && (d).m3 == 0.0f && (d).m4 == 0.0f) \
    PANIC("control decision on uninitialized motor outputs"); } while(0)

// ==========================================
// FLIGHT PHASE ENUM
// ==========================================
enum FlightPhase {
  PHASE_IDLE,
  PHASE_HOLD,
  PHASE_MISSION,
  PHASE_HOVER_SETTLE,
  PHASE_LANDING,
  PHASE_LANDED
};

const char* phaseName(FlightPhase p) {
  switch (p) {
    case PHASE_IDLE:         return "IDLE";
    case PHASE_HOLD:         return "HOLD";
    case PHASE_MISSION:      return "MISSION";
    case PHASE_HOVER_SETTLE: return "HOVER_SETTLE";
    case PHASE_LANDING:      return "LANDING";
    case PHASE_LANDED:       return "LANDED";
  }
  PANIC("phaseName: unhandled FlightPhase");
  return nullptr;
}

bool phaseFlightEnabled(FlightPhase phase) {
  return phase != PHASE_IDLE && phase != PHASE_LANDED;
}

// ==========================================
// RAW SENSOR READS
// What the hardware gives us every tick, before any phase logic
// touches it. Both tasks read from here; only the sensor-polling
// code writes to it.
// ==========================================
struct RawImuReading {
  float accX = 0.0f, accY = 0.0f, accZ = 0.0f;
  float gyroX = 0.0f, gyroY = 0.0f, gyroZ = 0.0f;
  float temp = 0.0f;
};

struct RawGpsReading {
  double lat = 0.0, lon = 0.0;
  bool   fix = false;
  int    sats = 0;
  float  speedMps = 0.0f;
  unsigned long lastFixMs = 0;
};

struct RawSensors {
  RawImuReading imu;
  RawGpsReading gps;
  float compassHeadingDeg = 0.0f;
  float baroAltitudeFt    = 0.0f; // baro read + ground-offset applied
};

// ==========================================
// PER-PHASE STATE BLOCKS
// Each phase owns three structs:
//   Dashboard_*  — what the sensors + nav math currently observe
//                  for this phase (read-only, written by tasks)
//   Cruise_*     — setpoints this phase is commanding (written by
//                  transitionTo and the nav/physics tick handlers)
//   Trip_*       — bookkeeping specific to this phase (counters,
//                  clocks, flags, captured positions)
//
// Only the structs for the CURRENT phase are meaningful at runtime.
// The others hold stale data from the last time that phase ran.
// ==========================================

// ------------------------------------------
// PHASE: IDLE
// Motors off, drone parked. No guidance, no clocks.
// ------------------------------------------
struct Dashboard_Idle {
  float altitudeFt  = 0.0f;
  float roll        = 0.0f;
  float pitch       = 0.0f;
  float yaw         = 0.0f;
};

struct Cruise_Idle {
  // No setpoints — motors are off.
};

struct Trip_Idle {
  // Nothing to track while parked.
};

// ------------------------------------------
// PHASE: HOLD
// Hovering in place after a mission abort.
// Maintains altitude and heading; no lateral guidance.
// ------------------------------------------
struct Dashboard_Hold {
  float altitudeFt      = 0.0f;
  float roll            = 0.0f;
  float pitch           = 0.0f;
  float yaw             = 0.0f;
  float compassHeading  = 0.0f;
  // Motor outputs — what the ESCs are currently commanded
  float m1 = 0, m2 = 0, m3 = 0, m4 = 0;
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
  unsigned long armedAtMs = 0; // carries over from MISSION on abort
};

// ------------------------------------------
// PHASE: MISSION
// Autonomous waypoint navigation.
// ------------------------------------------
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
  float  distToWP        = 0.0f; // computed each nav tick — telemetry
  float  bearingToWP     = 0.0f; // computed each nav tick — telemetry
  // Motor outputs — what the ESCs are currently commanded
  float m1 = 0, m2 = 0, m3 = 0, m4 = 0;
  float baseThrottle    = 0.0f;
  float rollCorrection  = 0.0f;
  float pitchCorrection = 0.0f;
};

struct Cruise_Mission {
  float targetAltFt      = 10.0f;
  float targetRollDeg    = 0.0f;
  float targetPitchDeg   = 0.0f;
  float yawTargetHeading = 0.0f;
};

struct Trip_Mission {
  unsigned long armedAtMs      = 0;
  int  currentWP               = 0;
  int  waypointCount           = 0;
  bool geofenceTripped         = false;
  bool gpsLossTripped          = false;
  bool active                  = false; // horizontal guidance running
  double launchLat             = 0.0;
  double launchLon             = 0.0;
  bool   launchPointSet        = false;
};

// ------------------------------------------
// PHASE: HOVER_SETTLE
// Brief level hover after mission complete, before landing.
// Holds altitude and heading; no lateral guidance.
// ------------------------------------------
struct Dashboard_HoverSettle {
  float altitudeFt     = 0.0f;
  float roll           = 0.0f;
  float pitch          = 0.0f;
  float yaw            = 0.0f;
  float compassHeading = 0.0f;
  // Motor outputs — what the ESCs are currently commanded
  float m1 = 0, m2 = 0, m3 = 0, m4 = 0;
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
  unsigned long enteredAtMs = 0; // times the settle dwell
};

// ------------------------------------------
// PHASE: LANDING
// Controlled descent to the ground.
// targetAltFt ramps down each nav tick until zero.
// ------------------------------------------
struct Dashboard_Landing {
  float altitudeFt     = 0.0f;
  float roll           = 0.0f;
  float pitch          = 0.0f;
  float yaw            = 0.0f;
  float compassHeading = 0.0f;
  // Motor outputs — what the ESCs are currently commanded
  float m1 = 0, m2 = 0, m3 = 0, m4 = 0;
  float baseThrottle    = 0.0f;
  float rollCorrection  = 0.0f;
  float pitchCorrection = 0.0f;
};

struct Cruise_Landing {
  float targetAltFt      = 0.0f; // ramped down by nav tick
  float targetRollDeg    = 0.0f;
  float targetPitchDeg   = 0.0f;
  float yawTargetHeading = 0.0f;
};

struct Trip_Landing {
  // No extra bookkeeping — the ramp lives in Cruise_Landing.targetAltFt.
};

// ------------------------------------------
// PHASE: LANDED
// On the ground, motors disarmed.
// ------------------------------------------
struct Dashboard_Landed {
  float altitudeFt = 0.0f;
};

struct Cruise_Landed {
  // Motors off — no setpoints.
};

struct Trip_Landed {
  // Nothing to track.
};

// ==========================================
// AGGREGATE SHARED STATE
// One instance of every per-phase triple lives here.
// The active phase determines which triple is live.
// ==========================================
struct SharedState {
  FlightPhase phase = PHASE_IDLE;

  // Raw hardware reads — always live, phase-agnostic
  RawSensors raw;

  // Per-phase triples
  Dashboard_Idle        dashboard_idle;
  Cruise_Idle           cruise_idle;
  Trip_Idle             trip_idle;

  Dashboard_Hold        dashboard_hold;
  Cruise_Hold           cruise_hold;
  Trip_Hold             trip_hold;

  Dashboard_Mission     dashboard_mission;
  Cruise_Mission        cruise_mission;
  Trip_Mission          trip_mission;

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

// ==========================================
// SHARED STATE INSTANCE + SYNCHRONISATION
// ==========================================
SemaphoreHandle_t sharedDataMutex;
SemaphoreHandle_t serialMutex;

volatile SharedState shared;

float groundAltitudeFt = 0;

JsonDocument readings;

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
// Pure functions — no shared state access.
// ==========================================
float gpsDistanceMeters(double lat1, double lon1, double lat2, double lon2) {
  const float R = 6371000.0f;
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  float a = sin(dLat/2)*sin(dLat/2) +
            cos(radians(lat1))*cos(radians(lat2))*
            sin(dLon/2)*sin(dLon/2);
  return R * 2.0f * atan2(sqrt(a), sqrt(1.0f - a));
}

float gpsBearing(double lat1, double lon1, double lat2, double lon2) {
  float dLon = radians(lon2 - lon1);
  float y    = sin(dLon) * cos(radians(lat2));
  float x    = cos(radians(lat1)) * sin(radians(lat2)) -
               sin(radians(lat1)) * cos(radians(lat2)) * cos(dLon);
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
// Same mixing law for every armed phase.
// ==========================================
struct MotorMix {
  float m1, m2, m3, m4;
  float baseThrottle, rollCorrection, pitchCorrection;
};

MotorMix computeMotorMix(float targetAltFt, float targetRollDeg, float targetPitchDeg,
                          float yawTargetHeading,
                          float altFt, float roll, float pitch, float compassHeading,
                          float dt) {
  MotorMix out;
  out.baseThrottle    = altitudePID.compute(targetAltFt,    altFt,         dt);
  out.rollCorrection  = rollPID.compute    (targetRollDeg,  roll,          dt);
  out.pitchCorrection = pitchPID.compute   (targetPitchDeg, pitch,         dt);

  float yawError = yawTargetHeading - compassHeading;
  if (yawError >  180.0f) yawError -= 360.0f;
  if (yawError < -180.0f) yawError += 360.0f;
  float yawCorr = yawPID.computeWithError(yawError, dt);

  out.m1 = constrain(out.baseThrottle + out.pitchCorrection + out.rollCorrection - yawCorr, 0.0f, 1.0f);
  out.m2 = constrain(out.baseThrottle + out.pitchCorrection - out.rollCorrection + yawCorr, 0.0f, 1.0f);
  out.m3 = constrain(out.baseThrottle - out.pitchCorrection + out.rollCorrection + yawCorr, 0.0f, 1.0f);
  out.m4 = constrain(out.baseThrottle - out.pitchCorrection - out.rollCorrection - yawCorr, 0.0f, 1.0f);
  return out;
}

// ==========================================
// PHASE TRANSITIONS
// One function owns every phase jump. Initialises the incoming
// phase's Trip_* bookkeeping and Cruise_* setpoints. Carry-overs
// between phases are explicit here.
//
// NOTE: takes the mutex itself — never call from inside withMutex().
// ==========================================
void transitionTo(FlightPhase next) {
  withMutex([&]() {
    switch (next) {

      case PHASE_IDLE:
        // No setpoints — motors will be off.
        break;

      case PHASE_HOLD:
        // Only entered via /abort from MISSION.
        // Carry armedAtMs so the flight-time clock doesn't reset.
        // Keep targetAltFt from wherever MISSION left it so altitude
        // doesn't jump on abort.
        shared.trip_hold.armedAtMs          = shared.trip_mission.armedAtMs;
        shared.cruise_hold.targetAltFt      = shared.cruise_mission.targetAltFt;
        shared.cruise_hold.yawTargetHeading = shared.cruise_mission.yawTargetHeading;
        shared.cruise_hold.targetRollDeg    = 0.0f;
        shared.cruise_hold.targetPitchDeg   = 0.0f;
        // Zero motor outputs — physics task hasn't run for this phase yet.
        // Any control-path read before the first physics tick will PANIC via
        // ASSERT_MOTORS_INITIALIZED.
        shared.dashboard_hold.m1 = shared.dashboard_hold.m2 = 0.0f;
        shared.dashboard_hold.m3 = shared.dashboard_hold.m4 = 0.0f;
        shared.dashboard_hold.baseThrottle = shared.dashboard_hold.rollCorrection = 0.0f;
        shared.dashboard_hold.pitchCorrection = 0.0f;
        break;

      case PHASE_MISSION:
        shared.trip_mission.armedAtMs       = millis();
        shared.trip_mission.currentWP       = 0;
        shared.trip_mission.waypointCount   = WAYPOINT_COUNT + 1; // +1 for RTL leg
        shared.trip_mission.geofenceTripped = false;
        shared.trip_mission.gpsLossTripped  = false;
        shared.trip_mission.active          = true;
        shared.trip_mission.launchLat       = shared.raw.gps.lat;
        shared.trip_mission.launchLon       = shared.raw.gps.lon;
        shared.trip_mission.launchPointSet  = true;
        shared.cruise_mission.targetAltFt      = WAYPOINTS[0].altFt;
        shared.cruise_mission.yawTargetHeading = shared.raw.compassHeadingDeg;
        shared.cruise_mission.targetRollDeg    = 0.0f;
        shared.cruise_mission.targetPitchDeg   = 0.0f;
        // Zero motor outputs — physics task hasn't run for this phase yet.
        shared.dashboard_mission.m1 = shared.dashboard_mission.m2 = 0.0f;
        shared.dashboard_mission.m3 = shared.dashboard_mission.m4 = 0.0f;
        shared.dashboard_mission.baseThrottle = shared.dashboard_mission.rollCorrection = 0.0f;
        shared.dashboard_mission.pitchCorrection = 0.0f;
        break;

      case PHASE_HOVER_SETTLE:
        shared.trip_hoverSettle.enteredAtMs        = millis();
        // Hold the altitude and heading MISSION left us at.
        shared.cruise_hoverSettle.targetAltFt      = shared.cruise_mission.targetAltFt;
        shared.cruise_hoverSettle.yawTargetHeading = shared.cruise_mission.yawTargetHeading;
        shared.cruise_hoverSettle.targetRollDeg    = 0.0f;
        shared.cruise_hoverSettle.targetPitchDeg   = 0.0f;
        // Zero motor outputs — physics task hasn't run for this phase yet.
        shared.dashboard_hoverSettle.m1 = shared.dashboard_hoverSettle.m2 = 0.0f;
        shared.dashboard_hoverSettle.m3 = shared.dashboard_hoverSettle.m4 = 0.0f;
        shared.dashboard_hoverSettle.baseThrottle = shared.dashboard_hoverSettle.rollCorrection = 0.0f;
        shared.dashboard_hoverSettle.pitchCorrection = 0.0f;
        break;

      case PHASE_LANDING:
        // Descent ramp starts from the altitude the previous phase held.
        // We don't know which phase is outgoing here, so read the raw
        // sensor altitude rather than guessing which Cruise_* to copy.
        shared.cruise_landing.targetAltFt      = shared.raw.baroAltitudeFt;
        shared.cruise_landing.yawTargetHeading = shared.cruise_hoverSettle.yawTargetHeading;
        shared.cruise_landing.targetRollDeg    = 0.0f;
        shared.cruise_landing.targetPitchDeg   = 0.0f;
        // Zero motor outputs — physics task hasn't run for this phase yet.
        shared.dashboard_landing.m1 = shared.dashboard_landing.m2 = 0.0f;
        shared.dashboard_landing.m3 = shared.dashboard_landing.m4 = 0.0f;
        shared.dashboard_landing.baseThrottle = shared.dashboard_landing.rollCorrection = 0.0f;
        shared.dashboard_landing.pitchCorrection = 0.0f;
        break;

      case PHASE_LANDED:
        // No setpoints — motors off.
        break;
    }
    shared.phase = next;
  });
}

// ==========================================
// PER-PHASE NAV TICK HANDLERS (~10 Hz)
// Each function runs for exactly one phase. The nav loop dispatches
// here after updating raw sensor state. Reads from raw.*, writes to
// the matching dashboard_*/cruise_*/trip_* structs.
// ==========================================

void navTick_Idle(float /*navDt*/) {
  withMutex([&]() {
    shared.dashboard_idle.altitudeFt = shared.raw.baroAltitudeFt;
    shared.dashboard_idle.roll       = shared.raw.imu.gyroX; // fused by physics
    shared.dashboard_idle.pitch      = shared.raw.imu.gyroY;
    shared.dashboard_idle.yaw        = shared.raw.imu.gyroZ;
  });
}

void navTick_Hold(float /*navDt*/) {
  withMutex([&]() {
    shared.dashboard_hold.altitudeFt     = shared.raw.baroAltitudeFt;
    shared.dashboard_hold.compassHeading = shared.raw.compassHeadingDeg;
    // roll/pitch/yaw come from physics task; read them back from cruise output
    shared.dashboard_hold.roll  = shared.raw.imu.gyroX;
    shared.dashboard_hold.pitch = shared.raw.imu.gyroY;
    shared.dashboard_hold.yaw   = shared.raw.imu.gyroZ;
  });

  // Safety: max flight time
  unsigned long armedAt;
  withMutex([&]() { armedAt = shared.trip_hold.armedAtMs; });
  if (armedAt > 0 && (millis() - armedAt) >= MAX_FLIGHT_TIME_MS) {
    logLine("[SAFETY] Max flight time reached in HOLD — forcing landing.");
    transitionTo(PHASE_LANDING);
  }
}

void navTick_Mission(float navDt) {
  // Snapshot raw sensors
  RawGpsReading  gps;
  Trip_Mission   trip;
  float compass;
  withMutex([&]() {
    gps     = shared.raw.gps;
    trip    = shared.trip_mission;
    compass = shared.raw.compassHeadingDeg;
  });

  // Update mission dashboard
  withMutex([&]() {
    shared.dashboard_mission.altitudeFt     = shared.raw.baroAltitudeFt;
    shared.dashboard_mission.compassHeading = compass;
    shared.dashboard_mission.gpsLat         = gps.lat;
    shared.dashboard_mission.gpsLon         = gps.lon;
    shared.dashboard_mission.gpsFix         = gps.fix;
    shared.dashboard_mission.gpsSats        = gps.sats;
    shared.dashboard_mission.roll           = shared.raw.imu.gyroX;
    shared.dashboard_mission.pitch          = shared.raw.imu.gyroY;
    shared.dashboard_mission.yaw            = shared.raw.imu.gyroZ;
  });

  // Safety: max flight time
  if (trip.armedAtMs > 0 && (millis() - trip.armedAtMs) >= MAX_FLIGHT_TIME_MS) {
    logLine("[SAFETY] Max flight time reached in MISSION — forcing landing.");
    withMutex([&]() {
      shared.trip_mission.active           = false;
      shared.cruise_mission.targetRollDeg  = 0.0f;
      shared.cruise_mission.targetPitchDeg = 0.0f;
    });
    transitionTo(PHASE_LANDING);
    return;
  }

  // Safety: geofence
  if (gps.fix && trip.launchPointSet) {
    float distFromLaunch = gpsDistanceMeters(gps.lat, gps.lon, trip.launchLat, trip.launchLon);
    if (distFromLaunch > GEOFENCE_RADIUS_M) {
      if (!trip.geofenceTripped) {
        withMutex([&]() { shared.trip_mission.geofenceTripped = true; });
        logLine("[SAFETY] Geofence exceeded — aborting mission.");
      }
      withMutex([&]() {
        shared.trip_mission.active           = false;
        shared.cruise_mission.targetRollDeg  = 0.0f;
        shared.cruise_mission.targetPitchDeg = 0.0f;
      });
      transitionTo(PHASE_LANDING);
      return;
    }
  }

  // Safety: GPS loss
  if (!gps.fix) {
    if (gps.lastFixMs > 0 && (millis() - gps.lastFixMs) >= GPS_LOSS_ABORT_MS) {
      if (!trip.gpsLossTripped) {
        withMutex([&]() { shared.trip_mission.gpsLossTripped = true; });
        logLine("[SAFETY] GPS fix lost — aborting mission.");
      }
      withMutex([&]() {
        shared.trip_mission.active           = false;
        shared.cruise_mission.targetRollDeg  = 0.0f;
        shared.cruise_mission.targetPitchDeg = 0.0f;
      });
      transitionTo(PHASE_LANDING);
      return;
    }
    // GPS not yet lost long enough — hold current setpoints
    return;
  }

  if (!trip.active) return;

  // All waypoints done (including RTL leg)
  if (trip.currentWP >= trip.waypointCount) {
    withMutex([&]() { shared.trip_mission.active = false; });
    transitionTo(PHASE_HOVER_SETTLE);
    logLine("[NAV] Mission complete (incl. RTL) — hovering before landing.");
    return;
  }

  Waypoint wp = getMissionWaypoint(trip.currentWP, trip.launchLat, trip.launchLon);
  float distM   = gpsDistanceMeters(gps.lat, gps.lon, wp.lat, wp.lon);
  float bearing = gpsBearing(gps.lat, gps.lon, wp.lat, wp.lon);

  withMutex([&]() {
    shared.cruise_mission.yawTargetHeading   = bearing;
    shared.dashboard_mission.distToWP        = distM;
    shared.dashboard_mission.bearingToWP     = bearing;
  });

  if (distM < WAYPOINT_ACCEPT_RADIUS_M) {
    bool isRTL = (trip.currentWP == WAYPOINT_COUNT);
    logLine(String("[NAV] Reached ") +
            (isRTL ? "RTL / launch point" : (String("waypoint ") + String(trip.currentWP))) +
            " — advancing to index " + String(trip.currentWP + 1));
    withMutex([&]() {
      shared.trip_mission.currentWP++;
      int next = shared.trip_mission.currentWP;
      shared.cruise_mission.targetAltFt = (next < shared.trip_mission.waypointCount)
          ? getMissionWaypoint(next, trip.launchLat, trip.launchLon).altFt
          : wp.altFt;
    });
    return;
  }

  float northM, eastM;
  bearingToNorthEast(distM, bearing, northM, eastM);
  float targetPitch = navNorthPID.computeWithError(northM, navDt);
  float targetRoll  = navEastPID.computeWithError(eastM,  navDt);

  withMutex([&]() {
    shared.cruise_mission.targetRollDeg  = targetRoll;
    shared.cruise_mission.targetPitchDeg = targetPitch;
  });
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
  withMutex([&]() {
    shared.dashboard_landed.altitudeFt = shared.raw.baroAltitudeFt;
  });
}

// ==========================================
// PER-PHASE PHYSICS TICK HANDLERS (~200 Hz)
// Each function runs for exactly one phase. Reads from raw.* and the
// phase's Cruise_* setpoints; writes the motor mix into Dashboard_*.
// ==========================================

void physicsTick_Idle(float dt) {
  (void)dt;
  altitudePID.reset();
  rollPID.reset();
  pitchPID.reset();
  yawPID.reset();
  navNorthPID.reset();
  navEastPID.reset();
  // esc1.disarm(); esc2.disarm(); esc3.disarm(); esc4.disarm();
}

void physicsTick_Hold(float dt) {
  Cruise_Hold c;
  RawSensors  r;
  withMutex([&]() { c = shared.cruise_hold; r = shared.raw; });

  MotorMix mix = computeMotorMix(
    c.targetAltFt, c.targetRollDeg, c.targetPitchDeg, c.yawTargetHeading,
    r.baroAltitudeFt, r.imu.gyroX, r.imu.gyroY, r.compassHeadingDeg, dt);

  withMutex([&]() {
    shared.dashboard_hold.m1             = mix.m1;
    shared.dashboard_hold.m2             = mix.m2;
    shared.dashboard_hold.m3             = mix.m3;
    shared.dashboard_hold.m4             = mix.m4;
    shared.dashboard_hold.baseThrottle   = mix.baseThrottle;
    shared.dashboard_hold.rollCorrection  = mix.rollCorrection;
    shared.dashboard_hold.pitchCorrection = mix.pitchCorrection;
  });
  // esc1.write(mix.m1); esc2.write(mix.m2); esc3.write(mix.m3); esc4.write(mix.m4);
}

void physicsTick_Mission(float dt) {
  Cruise_Mission c;
  RawSensors     r;
  withMutex([&]() { c = shared.cruise_mission; r = shared.raw; });

  MotorMix mix = computeMotorMix(
    c.targetAltFt, c.targetRollDeg, c.targetPitchDeg, c.yawTargetHeading,
    r.baroAltitudeFt, r.imu.gyroX, r.imu.gyroY, r.compassHeadingDeg, dt);

  withMutex([&]() {
    shared.dashboard_mission.m1             = mix.m1;
    shared.dashboard_mission.m2             = mix.m2;
    shared.dashboard_mission.m3             = mix.m3;
    shared.dashboard_mission.m4             = mix.m4;
    shared.dashboard_mission.baseThrottle   = mix.baseThrottle;
    shared.dashboard_mission.rollCorrection  = mix.rollCorrection;
    shared.dashboard_mission.pitchCorrection = mix.pitchCorrection;
  });
  // esc1.write(mix.m1); esc2.write(mix.m2); esc3.write(mix.m3); esc4.write(mix.m4);
}

void physicsTick_HoverSettle(float dt) {
  Cruise_HoverSettle c;
  RawSensors         r;
  withMutex([&]() { c = shared.cruise_hoverSettle; r = shared.raw; });

  MotorMix mix = computeMotorMix(
    c.targetAltFt, c.targetRollDeg, c.targetPitchDeg, c.yawTargetHeading,
    r.baroAltitudeFt, r.imu.gyroX, r.imu.gyroY, r.compassHeadingDeg, dt);

  withMutex([&]() {
    shared.dashboard_hoverSettle.m1             = mix.m1;
    shared.dashboard_hoverSettle.m2             = mix.m2;
    shared.dashboard_hoverSettle.m3             = mix.m3;
    shared.dashboard_hoverSettle.m4             = mix.m4;
    shared.dashboard_hoverSettle.baseThrottle   = mix.baseThrottle;
    shared.dashboard_hoverSettle.rollCorrection  = mix.rollCorrection;
    shared.dashboard_hoverSettle.pitchCorrection = mix.pitchCorrection;
  });
  // esc1.write(mix.m1); esc2.write(mix.m2); esc3.write(mix.m3); esc4.write(mix.m4);
}

void physicsTick_Landing(float dt) {
  Cruise_Landing c;
  RawSensors     r;
  withMutex([&]() { c = shared.cruise_landing; r = shared.raw; });

  MotorMix mix = computeMotorMix(
    c.targetAltFt, c.targetRollDeg, c.targetPitchDeg, c.yawTargetHeading,
    r.baroAltitudeFt, r.imu.gyroX, r.imu.gyroY, r.compassHeadingDeg, dt);

  withMutex([&]() {
    shared.dashboard_landing.m1             = mix.m1;
    shared.dashboard_landing.m2             = mix.m2;
    shared.dashboard_landing.m3             = mix.m3;
    shared.dashboard_landing.m4             = mix.m4;
    shared.dashboard_landing.baseThrottle   = mix.baseThrottle;
    shared.dashboard_landing.rollCorrection  = mix.rollCorrection;
    shared.dashboard_landing.pitchCorrection = mix.pitchCorrection;
  });
  // esc1.write(mix.m1); esc2.write(mix.m2); esc3.write(mix.m3); esc4.write(mix.m4);
}

void physicsTick_Landed(float dt) {
  (void)dt;
  altitudePID.reset();
  rollPID.reset();
  pitchPID.reset();
  yawPID.reset();
  navNorthPID.reset();
  navEastPID.reset();
  // esc1.disarm(); esc2.disarm(); esc3.disarm(); esc4.disarm();
}

// ==========================================
// CORE 0: NAVIGATION TASK (~10 Hz)
// Polls GPS + compass, updates raw sensors, dispatches to the
// per-phase nav tick handler for the current phase.
// ==========================================
void navigationTask(void* parameter) {
  const TickType_t xFrequency  = pdMS_TO_TICKS(NAV_LOOP_MS);
  TickType_t       lastWakeTime = xTaskGetTickCount();
  const float      navDt        = NAV_LOOP_MS / 1000.0f;

  for (;;) {
    // ---- Poll GPS ----
#ifdef WOKWI_SIM
    withMutex([&]() {
      if (shared.raw.gps.fix) shared.raw.gps.lastFixMs = millis();
      shared.raw.gps.sats = shared.raw.gps.fix ? 8 : 0;
    });
#else
    while (Serial2.available() > 0) gps.encode(Serial2.read());

    if (gps.location.isValid() && gps.location.age() < 2000) {
      RawGpsReading g;
      g.lat       = gps.location.lat();
      g.lon       = gps.location.lng();
      g.fix       = true;
      g.sats      = gps.satellites.value();
      g.speedMps  = gps.speed.mps();
      g.lastFixMs = millis();
      withMutex([&]() { shared.raw.gps = g; });
    } else {
      withMutex([&]() { shared.raw.gps.fix = false; });
    }

    // ---- Poll compass ----
    compass.read();
    float heading = compass.getAzimuth();
    withMutex([&]() { shared.raw.compassHeadingDeg = heading; });
#endif

    // ---- Dispatch to per-phase nav handler ----
    FlightPhase phase;
    withMutex([&]() { phase = shared.phase; });

    switch (phase) {
      case PHASE_IDLE:         navTick_Idle(navDt);        break;
      case PHASE_HOLD:         navTick_Hold(navDt);        break;
      case PHASE_MISSION:      navTick_Mission(navDt);     break;
      case PHASE_HOVER_SETTLE: navTick_HoverSettle(navDt); break;
      case PHASE_LANDING:      navTick_Landing(navDt);     break;
      case PHASE_LANDED:       navTick_Landed(navDt);      break;
      default: PANIC("navigationTask: unhandled FlightPhase");
    }

    vTaskDelayUntil(&lastWakeTime, xFrequency);
  }
}

// ==========================================
// CORE 1: PHYSICS + FLIGHT TASK (~200 Hz)
// Reads IMU + baro, updates raw sensors, dispatches to the
// per-phase physics tick handler for the current phase.
// ==========================================
unsigned long lastGyroMicros = 0;

void physicsTask(void* parameter) {
  lastGyroMicros = micros();
  filter.begin(PHYSICS_LOOP_HZ);
  const TickType_t xFrequency  = pdMS_TO_TICKS(PHYSICS_LOOP_MS);
  TickType_t       lastWakeTime = xTaskGetTickCount();

  for (;;) {
    unsigned long now = micros();
    float dt = (now - lastGyroMicros) / 1000000.0f;
    lastGyroMicros = now;

    // ---- Read IMU ----
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float gx = g.gyro.x * 57.2958f;
    float gy = g.gyro.y * 57.2958f;
    float gz = g.gyro.z * 57.2958f;

    if (dt > 0 && dt < 1.0f) {
      filter.updateIMU(gx, gy, gz, a.acceleration.x, a.acceleration.y, a.acceleration.z);
    }

    float baroAlt = ((float)barometer.readAltitudeMeters() * 3.28084f) - groundAltitudeFt;

    withMutex([&]() {
      shared.raw.imu.gyroX  = filter.getRoll();
      shared.raw.imu.gyroY  = filter.getPitch();
      shared.raw.imu.gyroZ  = filter.getYaw();
      shared.raw.imu.accX   = a.acceleration.x;
      shared.raw.imu.accY   = a.acceleration.y;
      shared.raw.imu.accZ   = a.acceleration.z;
      shared.raw.imu.temp   = temp.temperature;
      shared.raw.baroAltitudeFt = baroAlt;
    });

    // ---- Dispatch to per-phase physics handler ----
    FlightPhase phase;
    withMutex([&]() { phase = shared.phase; });

    switch (phase) {
      case PHASE_IDLE:         physicsTick_Idle(dt);        break;
      case PHASE_HOLD:         physicsTick_Hold(dt);        break;
      case PHASE_MISSION:      physicsTick_Mission(dt);     break;
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
// Grab a snapshot of whichever phase structs the front-end needs.
// ==========================================
String getGyroReadings() {
  float roll, pitch, yaw;
  withMutex([&]() {
    roll  = shared.raw.imu.gyroX;
    pitch = shared.raw.imu.gyroY;
    yaw   = shared.raw.imu.gyroZ;
  });
  readings["gyroX"] = roll;
  readings["gyroY"] = pitch;
  readings["gyroZ"] = yaw;
  String out; serializeJson(readings, out); return out;
}

String getAccReadings() {
  float ax, ay, az;
  withMutex([&]() {
    ax = shared.raw.imu.accX;
    ay = shared.raw.imu.accY;
    az = shared.raw.imu.accZ;
  });
  readings["accX"] = ax;
  readings["accY"] = ay;
  readings["accZ"] = az;
  String out; serializeJson(readings, out); return out;
}

String getFlightReadings() {
  FlightPhase phase;
  withMutex([&]() { phase = shared.phase; });

  // Fields common to all phases
  readings["flightPhase"]   = phaseName(phase);
  readings["flightEnabled"] = phaseFlightEnabled(phase);
  readings["altFt"]         = shared.raw.baroAltitudeFt;

  // Phase-specific snapshot
  switch (phase) {
    case PHASE_IDLE: {
      withMutex([&]() {
        readings["targetFt"] = 0.0f;
        readings["m1"] = 0; readings["m2"] = 0; readings["m3"] = 0; readings["m4"] = 0;
        readings["baseThrottle"] = 0; readings["rollCorrection"] = 0; readings["pitchCorrection"] = 0;
        readings["gpsFix"] = false; readings["gpsLat"] = 0.0; readings["gpsLon"] = 0.0;
        readings["gpsSats"] = 0; readings["compassHeading"] = 0.0;
        readings["navActive"] = false; readings["navWaypoint"] = 0; readings["navWaypointCount"] = 0;
        readings["navDistM"] = 0.0; readings["navBearing"] = 0.0;
        readings["flightSecRemaining"] = (long)(MAX_FLIGHT_TIME_MS / 1000);
      });
      break;
    }
    case PHASE_HOLD: {
      Cruise_Hold    c;
      Dashboard_Hold db;
      Trip_Hold      t;
      RawGpsReading  g;
      withMutex([&]() { c = shared.cruise_hold; db = shared.dashboard_hold; t = shared.trip_hold; g = shared.raw.gps; });
      readings["targetFt"]        = c.targetAltFt;
      readings["m1"]              = (int)(db.m1 * 100);
      readings["m2"]              = (int)(db.m2 * 100);
      readings["m3"]              = (int)(db.m3 * 100);
      readings["m4"]              = (int)(db.m4 * 100);
      readings["baseThrottle"]    = (int)(db.baseThrottle   * 100);
      readings["rollCorrection"]  = (int)(db.rollCorrection  * 100);
      readings["pitchCorrection"] = (int)(db.pitchCorrection * 100);
      readings["compassHeading"]  = shared.raw.compassHeadingDeg;
      readings["gpsFix"]          = g.fix;
      readings["gpsLat"]          = g.lat;
      readings["gpsLon"]          = g.lon;
      readings["gpsSats"]         = g.sats;
      readings["navActive"]       = false;
      readings["navWaypoint"]     = 0; readings["navWaypointCount"] = 0;
      readings["navDistM"]        = 0.0; readings["navBearing"] = 0.0;
      readings["flightSecRemaining"] = t.armedAtMs > 0
          ? max(0L, (long)((MAX_FLIGHT_TIME_MS - (millis() - t.armedAtMs)) / 1000))
          : (long)(MAX_FLIGHT_TIME_MS / 1000);
      break;
    }
    case PHASE_MISSION: {
      Dashboard_Mission d;
      Cruise_Mission    c;
      Trip_Mission      t;
      withMutex([&]() { d = shared.dashboard_mission; c = shared.cruise_mission; t = shared.trip_mission; });
      readings["targetFt"]        = c.targetAltFt;
      readings["m1"]              = (int)(d.m1 * 100);
      readings["m2"]              = (int)(d.m2 * 100);
      readings["m3"]              = (int)(d.m3 * 100);
      readings["m4"]              = (int)(d.m4 * 100);
      readings["baseThrottle"]    = (int)(d.baseThrottle   * 100);
      readings["rollCorrection"]  = (int)(d.rollCorrection  * 100);
      readings["pitchCorrection"] = (int)(d.pitchCorrection * 100);
      readings["compassHeading"]  = d.compassHeading;
      readings["gpsFix"]          = d.gpsFix;
      readings["gpsLat"]          = d.gpsLat;
      readings["gpsLon"]          = d.gpsLon;
      readings["gpsSats"]         = d.gpsSats;
      readings["navActive"]       = t.active;
      readings["navWaypoint"]     = t.currentWP;
      readings["navWaypointCount"]= t.waypointCount;
      readings["navDistM"]        = d.distToWP;
      readings["navBearing"]      = d.bearingToWP;
      readings["flightSecRemaining"] = t.armedAtMs > 0
          ? max(0L, (long)((MAX_FLIGHT_TIME_MS - (millis() - t.armedAtMs)) / 1000))
          : (long)(MAX_FLIGHT_TIME_MS / 1000);
      break;
    }
    case PHASE_HOVER_SETTLE: {
      Cruise_HoverSettle    c;
      Dashboard_HoverSettle db;
      withMutex([&]() { c = shared.cruise_hoverSettle; db = shared.dashboard_hoverSettle; });
      readings["targetFt"]        = c.targetAltFt;
      readings["m1"]              = (int)(db.m1 * 100);
      readings["m2"]              = (int)(db.m2 * 100);
      readings["m3"]              = (int)(db.m3 * 100);
      readings["m4"]              = (int)(db.m4 * 100);
      readings["baseThrottle"]    = (int)(db.baseThrottle   * 100);
      readings["rollCorrection"]  = (int)(db.rollCorrection  * 100);
      readings["pitchCorrection"] = (int)(db.pitchCorrection * 100);
      readings["compassHeading"]  = shared.raw.compassHeadingDeg;
      readings["gpsFix"]          = shared.raw.gps.fix;
      readings["gpsLat"]          = shared.raw.gps.lat;
      readings["gpsLon"]          = shared.raw.gps.lon;
      readings["gpsSats"]         = shared.raw.gps.sats;
      readings["navActive"]       = false;
      readings["navWaypoint"]     = 0; readings["navWaypointCount"] = 0;
      readings["navDistM"]        = 0.0; readings["navBearing"] = 0.0;
      readings["flightSecRemaining"] = (long)(MAX_FLIGHT_TIME_MS / 1000);
      break;
    }
    case PHASE_LANDING: {
      Cruise_Landing    c;
      Dashboard_Landing db;
      withMutex([&]() { c = shared.cruise_landing; db = shared.dashboard_landing; });
      readings["targetFt"]        = c.targetAltFt;
      readings["m1"]              = (int)(db.m1 * 100);
      readings["m2"]              = (int)(db.m2 * 100);
      readings["m3"]              = (int)(db.m3 * 100);
      readings["m4"]              = (int)(db.m4 * 100);
      readings["baseThrottle"]    = (int)(db.baseThrottle   * 100);
      readings["rollCorrection"]  = (int)(db.rollCorrection  * 100);
      readings["pitchCorrection"] = (int)(db.pitchCorrection * 100);
      readings["compassHeading"]  = shared.raw.compassHeadingDeg;
      readings["gpsFix"]          = shared.raw.gps.fix;
      readings["gpsLat"]          = shared.raw.gps.lat;
      readings["gpsLon"]          = shared.raw.gps.lon;
      readings["gpsSats"]         = shared.raw.gps.sats;
      readings["navActive"]       = false;
      readings["navWaypoint"]     = 0; readings["navWaypointCount"] = 0;
      readings["navDistM"]        = 0.0; readings["navBearing"] = 0.0;
      readings["flightSecRemaining"] = (long)(MAX_FLIGHT_TIME_MS / 1000);
      break;
    }
    case PHASE_LANDED: {
      withMutex([&]() {
        readings["targetFt"] = 0.0f;
        readings["m1"] = 0; readings["m2"] = 0; readings["m3"] = 0; readings["m4"] = 0;
        readings["baseThrottle"] = 0; readings["rollCorrection"] = 0; readings["pitchCorrection"] = 0;
        readings["gpsFix"] = shared.raw.gps.fix;
        readings["gpsLat"] = shared.raw.gps.lat;
        readings["gpsLon"] = shared.raw.gps.lon;
        readings["gpsSats"] = shared.raw.gps.sats;
        readings["compassHeading"] = shared.raw.compassHeadingDeg;
        readings["navActive"] = false;
        readings["navWaypoint"] = 0; readings["navWaypointCount"] = 0;
        readings["navDistM"] = 0.0; readings["navBearing"] = 0.0;
        readings["flightSecRemaining"] = (long)(MAX_FLIGHT_TIME_MS / 1000);
      });
      break;
    }
    default: PANIC("getFlightReadings: unhandled FlightPhase");
  }

  String out; serializeJson(readings, out); return out;
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
  if (hasCal) logLine("[COMPASS] Loaded saved calibration from flash.");
  else         logLine("[COMPASS] WARNING: no saved calibration found.");
  compass.setCalibration(compassOffsetX, compassOffsetY, compassOffsetZ,
                          compassScaleX,  compassScaleY,  compassScaleZ);
}

void runCompassCalibration() {
  logLine("[COMPASS] Calibration starting — rotate drone slowly through all axes now...");
  int16_t minX = 32767, maxX = -32768;
  int16_t minY = 32767, maxY = -32768;
  int16_t minZ = 32767, maxZ = -32768;
  unsigned long start = millis();
  while (millis() - start < COMPASS_CAL_DURATION_MS) {
    compass.read();
    int16_t x = compass.getX(), y = compass.getY(), z = compass.getZ();
    minX = min(minX, x); maxX = max(maxX, x);
    minY = min(minY, y); maxY = max(maxY, y);
    minZ = min(minZ, z); maxZ = max(maxZ, z);
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
  compass.setCalibration(compassOffsetX, compassOffsetY, compassOffsetZ,
                          compassScaleX,  compassScaleY,  compassScaleZ);
  compassPrefs.begin("compass", false);
  compassPrefs.putFloat("offX", compassOffsetX); compassPrefs.putFloat("offY", compassOffsetY);
  compassPrefs.putFloat("offZ", compassOffsetZ); compassPrefs.putFloat("sclX", compassScaleX);
  compassPrefs.putFloat("sclY", compassScaleY);  compassPrefs.putFloat("sclZ", compassScaleZ);
  compassPrefs.end();
  logLine("[COMPASS] Calibration saved.");
}

void initCompass() {
  logLine("[COMPASS] Initializing QMC5883L...");
  compass.init();
  compass.setMode(0x01, 0x0C, 0x10, 0xC0);
  if (CALIBRATE_COMPASS_ON_BOOT) runCompassCalibration();
  else                            loadCompassCalibration();
  logLine("[COMPASS] QMC5883L ready.");
}

void initGPS() {
  logLine("[GPS] Initializing BN-880 GPS on Serial2...");
  Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
}

void initLittleFS() {
  if (!LittleFS.begin(false, "/littlefs", 10, "spiffs")) {
    LittleFS.format();
    LittleFS.begin(false, "/littlefs", 10, "spiffs");
  }
}

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  logLine("[WiFi] Connecting...");
  while (WiFi.status() != WL_CONNECTED) { delay(1000); }
  logLine(String("[WiFi] ") + WiFi.localIP().toString());
}

// ==========================================
// WOKWI SIM INPUT PARSER
// ==========================================
#ifdef WOKWI_SIM
void parseSimInput() {
  static String buf = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      if (buf.startsWith("HDG:")) {
        withMutex([&]() { shared.raw.compassHeadingDeg = buf.substring(4).toFloat(); });
      } else if (buf.startsWith("LAT:")) {
        withMutex([&]() { shared.raw.gps.lat = buf.substring(4).toDouble(); });
      } else if (buf.startsWith("LON:")) {
        withMutex([&]() { shared.raw.gps.lon = buf.substring(4).toDouble(); });
      } else if (buf.startsWith("FIX:")) {
        withMutex([&]() { shared.raw.gps.fix = (buf.substring(4).toInt() == 1); });
      } else if (buf.startsWith("MISSION:")) {
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
// SETUP
// ==========================================
void setup() {
  sharedDataMutex = xSemaphoreCreateMutex();
  serialMutex     = xSemaphoreCreateMutex();

  Serial.begin(115200);
  Wire.begin(16, 15);

  logLine("=== BOOT START ===");

  initWiFi();
  initLittleFS();
  initMPU();
#ifndef WOKWI_SIM
  initCompass();
  initGPS();
#endif
  barometer.initialize();
  // esc1.init(4, RMT_CHANNEL_0); esc2.init(5, RMT_CHANNEL_1);
  // esc3.init(6, RMT_CHANNEL_2); esc4.init(7, RMT_CHANNEL_3);
  logLine("[ESC] DShot600 ready on GPIO 4/5/6/7");

  groundAltitudeFt = (float)barometer.readAltitudeMeters() * 3.28084f;

  xTaskCreatePinnedToCore(navigationTask, "NavTask",     8192, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(physicsTask,    "PhysicsTask", 8192, NULL, 2, NULL, 1);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* r) {
    r->send(LittleFS, "/index.html", "text/html");
  });
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  server.on("/start-waypoint-nav", HTTP_GET, [](AsyncWebServerRequest* r) {
    bool hasFix;
    withMutex([&]() { hasFix = shared.raw.gps.fix; });
    if (!hasFix) {
      r->send(400, "text/plain", "NO GPS FIX — wait for satellite lock before starting mission");
      return;
    }
    transitionTo(PHASE_MISSION);
    logLine("[FLIGHT] Drone Armed & Mission Started. Launch point captured.");
    r->send(200, "text/plain", "WAYPOINT NAVIGATION STARTED");
  });

  server.on("/abort", HTTP_GET, [](AsyncWebServerRequest* r) {
    FlightPhase currentPhase;
    withMutex([&]() { currentPhase = shared.phase; });
    if (currentPhase == PHASE_MISSION) {
      withMutex([&]() {
        shared.trip_mission.active           = false;
        shared.cruise_mission.targetRollDeg  = 0.0f;
        shared.cruise_mission.targetPitchDeg = 0.0f;
      });
      transitionTo(PHASE_HOLD);
    }
    logLine("[MISSION] Mission aborted — holding position.");
    r->send(200, "text/plain", "MISSION ABORTED — HOLDING");
  });

  server.on("/land", HTTP_GET, [](AsyncWebServerRequest* r) {
    FlightPhase currentPhase;
    withMutex([&]() { currentPhase = shared.phase; });
    if (currentPhase == PHASE_MISSION) {
      withMutex([&]() {
        shared.trip_mission.active           = false;
        shared.cruise_mission.targetRollDeg  = 0.0f;
        shared.cruise_mission.targetPitchDeg = 0.0f;
      });
    }
    transitionTo(PHASE_LANDING);
    logLine("[FLIGHT] Manual landing requested.");
    r->send(200, "text/plain", "LANDING");
  });

  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest* r) {
    transitionTo(PHASE_IDLE);
    logLine("[FLIGHT] EMERGENCY STOP — motors cut.");
    r->send(200, "text/plain", "STOPPED");
  });

  server.on("/calibrate-compass", HTTP_GET, [](AsyncWebServerRequest* r) {
    bool enabled;
    withMutex([&]() { enabled = phaseFlightEnabled(shared.phase); });
    if (enabled) {
      r->send(400, "text/plain", "REFUSED — disarm first");
      return;
    }
    r->send(200, "text/plain", "CALIBRATING — rotate the drone slowly for 30 seconds");
    runCompassCalibration();
  });

  events.onConnect([](AsyncEventSourceClient* c) { c->send("hello!", NULL, millis(), 10000); });
  server.addHandler(&events);
  server.begin();

  logLine("[Web] Server started.");
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
  if (millis() - lastGyroSend > SSE_GYRO_MS) {
    events.send(getGyroReadings().c_str(), "gyro_readings", millis());
    lastGyroSend = millis();
  }
  if (millis() - lastAccSend > SSE_ACC_MS) {
    events.send(getAccReadings().c_str(), "accelerometer_readings", millis());
    lastAccSend = millis();
  }
  if (millis() - lastFlightSend > SSE_FLIGHT_MS) {
    events.send(getFlightReadings().c_str(), "flight_readings", millis());
    lastFlightSend = millis();
  }
}