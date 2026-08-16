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
const float PHYSICS_LOOP_HZ = 1000.0f / PHYSICS_LOOP_MS;
const unsigned long SSE_GYRO_MS    = 10;
const unsigned long SSE_ACC_MS     = 200;
const unsigned long SSE_FLIGHT_MS  = 100;
const unsigned long NAV_LOOP_MS    = 100;

// ==========================================
// SAFETY CONFIGURATION
// ==========================================
const unsigned long MAX_FLIGHT_TIME_MS = 5UL * 60UL * 1000UL; // 5 minutes
const float GEOFENCE_RADIUS_M = 150.0f;
const unsigned long GPS_LOSS_ABORT_MS = 3000;

// ==========================================
// BN-880 WIRING
// ==========================================
#define GPS_RX_PIN 17
#define GPS_TX_PIN 18
#define GPS_BAUD   9600

// ==========================================
// COMPASS CALIBRATION
// ==========================================
const bool CALIBRATE_COMPASS_ON_BOOT = false; 
const unsigned long COMPASS_CAL_DURATION_MS = 30000; 

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
const int WAYPOINT_COUNT = sizeof(WAYPOINTS) / sizeof(WAYPOINTS[0]);
const float WAYPOINT_ACCEPT_RADIUS_M = 5.0f;
const unsigned long MISSION_COMPLETE_HOVER_MS = 10000; 
const float LAND_DESCENT_RATE_FPS = 1.5f; 

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
Madgwick filter;
EspBarometer barometer;
TinyGPSPlus gps;           
QMC5883LCompass compass;   

AsyncWebServer server(80);
AsyncEventSource events("/events");

// ==========================================
// PID CONTROLLERS
// ==========================================
PID altitudePID (0.08f,  0.01f,  0.05f,  0.0f,   1.0f );
PID rollPID     (0.01f,  0.001f, 0.005f, -0.3f,  0.3f );
PID pitchPID    (0.01f,  0.001f, 0.005f, -0.3f,  0.3f );
PID yawPID      (0.005f, 0.0001f,0.001f, -0.2f,  0.2f );
PID navNorthPID (0.5f,   0.0f,   0.1f,   -15.0f, 15.0f); 
PID navEastPID  (0.5f,   0.0f,   0.1f,   -15.0f, 15.0f); 

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

// Halt with a message on Serial — embedded equivalent of an unhandled exception.
// Use anywhere a switch over FlightPhase hits a case that should be unreachable.
// Motors will stop (physicsTask stalls), which is the safe failure mode.
#define PANIC(msg) do { Serial.println(F("[PANIC] " msg " — halting")); while(1) { delay(10); } } while(0)

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
  return nullptr; // unreachable — suppresses compiler warning
}

// flightEnabled is not stored per-phase — it's a pure function of phase.
// IDLE and LANDED always disarm; every other phase always arms.
bool phaseFlightEnabled(FlightPhase phase) {
  return phase != PHASE_IDLE && phase != PHASE_LANDED;
}

// ==========================================
// DATA STRUCTURES — SENSORS (the gauges)
// Raw/fused sensor inputs — what the hardware is physically telling us
// right now. Nothing here is computed or commanded, only measured.
// Think of it as the gauges on a dashboard: .gps is where the nav map
// says you are, .imu is the tilt/altitude readout, .compassHeading is
// the compass.
// ==========================================
struct GpsData {
  double lat = 0.0;
  double lon = 0.0;
  bool fix = false;
  int sats = 0;
  float speedMps = 0.0f;
  unsigned long lastFixMs = 0;
};

struct ImuData {
  float roll = 0.0f;
  float pitch = 0.0f;
  float yaw = 0.0f;
  float accX = 0.0f;
  float accY = 0.0f;
  float accZ = 0.0f;
  float temp = 0.0f;
  float altitudeFt = 0.0f;
};

struct CarDashboardState {
  GpsData gps;
  ImuData imu;
  float compassHeading = 0.0f;
  float navDistToWP    = 0.0f;  // distance to active waypoint — computed by nav task, read-only telemetry
  float navBearingToWP = 0.0f;  // bearing to active waypoint — computed by nav task, read-only telemetry
};

// ==========================================
// DATA STRUCTURES — CRUISE CONTROL (the setpoints + actuator output)
// PID-computed guidance targets and resulting actuator commands.
// Not sensed — commanded. This is the dial you turned vs. what the
// speedometer reads.
//
// targetAltFt lives here alongside targetRollDeg / targetPitchDeg —
// all three are altitude/attitude setpoints the PIDs chase, set by
// transitionTo() when phases change and updated by navigationTask
// during MISSION (altitude per waypoint) and LANDING (ramp to zero).
//
// yawTargetHeading is also a setpoint: held across HOVER_SETTLE and
// LANDING deliberately, not reset on phase transition.
// ==========================================
struct CruiseControlState_NavData {
  float targetAltFt      = 10.0f;  // altitude setpoint — written by transitionTo() and nav task
  float targetRollDeg    = 0.0f;
  float targetPitchDeg   = 0.0f;
  float yawTargetHeading = 0.0f;
};

struct CruiseControlState_MotorData {
  float m1 = 0, m2 = 0, m3 = 0, m4 = 0;
  float baseThrottle    = 0;
  float rollCorrection  = 0;
  float pitchCorrection = 0;
};

struct CruiseControlState {
  CruiseControlState_NavData  nav;
  CruiseControlState_MotorData motors;
};

// ==========================================
// DATA STRUCTURES — FLIGHT PHASE STATE
//
// This is the road trip itself: which leg you're on, and the state
// that is genuinely specific to that leg and varies within it.
//
// Rules for what lives here vs. in CruiseControlState:
//   - Commanded setpoints (targetAltFt, targetRoll, targetPitch,
//     yawTargetHeading) → CruiseControlState. They are PID inputs,
//     not phase bookkeeping.
//   - Motor arm/disarm gate (flightEnabled) → phaseFlightEnabled(),
//     a pure function of the current phase enum. No per-struct field.
//   - Safety clocks and counters that are specific to one or two
//     phases and vary while active → here (armedAtMs, enteredAtMs,
//     currentWP, etc.).
//   - Mission-specific flags and captured launch position → MissionState.
//
// Only the struct matching the CURRENT phase is meaningful; others
// hold stale data from the last time that phase ran.
// ==========================================
struct IdleState {
  // Disarmed, motors off. Nothing to track.
};

struct HoldState {
  unsigned long armedAtMs = 0;   // safety clock — carries over from MISSION on abort
};

struct MissionState {
  unsigned long armedAtMs    = 0;
  int  currentWP             = 0;
  int  waypointCount         = 0;
  bool geofenceTripped       = false;
  bool gpsLossTripped        = false;
  bool active                = false;  // horizontal waypoint guidance running
  double launchLat           = 0.0;   // captured from GPS at MISSION start
  double launchLon           = 0.0;
  bool launchPointSet        = false;
};

struct HoverSettleState {
  unsigned long enteredAtMs  = 0;      // used to time the settle dwell
};

struct LandingState {
  // targetAltFt ramp lives in CruiseControlState.nav.targetAltFt.
  // Nothing phase-specific needed beyond the phase enum itself.
};

struct LandedState {
  // Disarmed. targetAltFt is 0 by convention (set in transitionTo).
  // phaseFlightEnabled(PHASE_LANDED) == false.
};

struct RoadTripState {
  FlightPhase    phase = PHASE_IDLE;
  IdleState      idle;
  HoldState      hold;
  MissionState   mission;
  HoverSettleState hoverSettle;
  LandingState   landing;
  LandedState    landed;
};

// Safety-clock lookup — the one place that knows which phase struct
// holds armedAtMs. Nothing else should reach into hold.armedAtMs or
// mission.armedAtMs directly.
// Only valid during HOLD and MISSION — calling this from any other phase
// is a bug. PANIC halts rather than returning a silent 0 that would
// corrupt the flight-time safety clock.
unsigned long currentArmedAtMs(const RoadTripState& trip) {
  switch (trip.phase) {
    case PHASE_HOLD:    return trip.hold.armedAtMs;
    case PHASE_MISSION: return trip.mission.armedAtMs;
    default:            PANIC("currentArmedAtMs: called from non-armed phase"); return 0;
  }
}

// ==========================================
// SHARED STATE VARIABLES
// ==========================================
SemaphoreHandle_t sharedDataMutex;
SemaphoreHandle_t serialMutex;  

volatile CarDashboardState  sharedDashboard;
volatile CruiseControlState sharedCruiseControl;
volatile RoadTripState      sharedTrip;

// One-time calibration baseline measured at boot — not a live reading.
float groundAltitudeFt = 0;

JsonDocument readings;

// ==========================================
// MUTEX & LOGGING HELPERS
// ==========================================
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
// PHASE TRANSITIONS
//
// One function owns every jump between flight phases.
// Commanded setpoints (targetAltFt, yawTargetHeading) are written
// directly into sharedCruiseControl here — no need for currentTargetAltFt()
// dispatch helpers. Phase-specific bookkeeping (armedAtMs, currentWP,
// launchLat/Lon, etc.) is initialized in the matching case.
//
// Carry-overs are explicit:
//   MISSION -> HOLD:    armedAtMs carries so the flight-time clock
//                       doesn't reset on abort.
//   HOVER_SETTLE/LANDING: targetAltFt is read from cruise control
//                       before phase flips so the drone doesn't jump.
//
// NOTE: takes the mutex itself — never call from inside withMutex().
//       sharedDataMutex is not recursive.
// ==========================================
void transitionTo(FlightPhase next) {
  withMutex([&]() {
    switch (next) {
      case PHASE_IDLE:
        // Motors off; targetAltFt doesn't matter but zero it for clarity.
        sharedCruiseControl.nav.targetAltFt    = 0.0f;
        sharedCruiseControl.nav.targetRollDeg  = 0.0f;
        sharedCruiseControl.nav.targetPitchDeg = 0.0f;
        break;

      case PHASE_HOLD:
        // Only reached via /abort (MISSION -> HOLD). Carry the safety
        // clock forward so aborting doesn't reset the flight timer.
        // targetAltFt is already set in cruise control — leave it alone
        // so the drone doesn't jump altitude on abort.
        sharedTrip.hold.armedAtMs = sharedTrip.mission.armedAtMs;
        sharedCruiseControl.nav.targetRollDeg  = 0.0f;
        sharedCruiseControl.nav.targetPitchDeg = 0.0f;
        break;

      case PHASE_MISSION:
        sharedTrip.mission.armedAtMs     = millis();
        sharedTrip.mission.currentWP     = 0;
        sharedTrip.mission.waypointCount = WAYPOINT_COUNT + 1; // +1 for RTL leg
        sharedTrip.mission.geofenceTripped = false;
        sharedTrip.mission.gpsLossTripped  = false;
        sharedTrip.mission.active          = true;
        // Launch point + starting heading captured straight from live
        // sensor state — this IS what "starting a mission" means.
        sharedTrip.mission.launchLat     = sharedDashboard.gps.lat;
        sharedTrip.mission.launchLon     = sharedDashboard.gps.lon;
        sharedTrip.mission.launchPointSet = true;
        sharedCruiseControl.nav.targetAltFt      = WAYPOINTS[0].altFt;
        sharedCruiseControl.nav.yawTargetHeading = sharedDashboard.compassHeading;
        break;

      case PHASE_HOVER_SETTLE:
        sharedTrip.hoverSettle.enteredAtMs = millis();
        // Hold whatever altitude the mission left us at — read from
        // cruise control before phase flips below.
        // (targetAltFt unchanged — no write needed.)
        sharedCruiseControl.nav.targetRollDeg  = 0.0f;
        sharedCruiseControl.nav.targetPitchDeg = 0.0f;
        break;

      case PHASE_LANDING:
        // Landing ramp starts from wherever cruise control currently
        // has targetAltFt — read before phase flips below.
        // (targetAltFt unchanged — nav task will ramp it down each tick.)
        sharedCruiseControl.nav.targetRollDeg  = 0.0f;
        sharedCruiseControl.nav.targetPitchDeg = 0.0f;
        break;

      case PHASE_LANDED:
        sharedCruiseControl.nav.targetAltFt   = 0.0f;
        sharedCruiseControl.nav.targetRollDeg  = 0.0f;
        sharedCruiseControl.nav.targetPitchDeg = 0.0f;
        break;
    }
    sharedTrip.phase = next;
  });
}

// ==========================================
// GPS / NAVIGATION HELPERS
// ==========================================
float gpsDistanceMeters(double lat1, double lon1, double lat2, double lon2) {
  const float R = 6371000.0f; 
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  float a = sin(dLat/2)*sin(dLat/2) +
            cos(radians(lat1))*cos(radians(lat2))*
            sin(dLon/2)*sin(dLon/2);
  return R * 2.0f * atan2(sqrt(a), sqrt(1.0f-a));
}

float gpsBearing(double lat1, double lon1, double lat2, double lon2) {
  float dLon = radians(lon2 - lon1);
  float y = sin(dLon) * cos(radians(lat2));
  float x = cos(radians(lat1)) * sin(radians(lat2)) -
            sin(radians(lat1)) * cos(radians(lat2)) * cos(dLon);
  float bearing = degrees(atan2(y, x));
  return fmod(bearing + 360.0f, 360.0f); 
}

void bearingToNorthEast(float distM, float bearingDeg, float &northM, float &eastM) {
  float bearingRad = radians(bearingDeg);
  northM = distM * cos(bearingRad);
  eastM  = distM * sin(bearingRad);
}

void stopNavigation() {
  withMutex([&]() {
    sharedTrip.mission.active = false;
    sharedCruiseControl.nav.targetRollDeg  = 0.0f;
    sharedCruiseControl.nav.targetPitchDeg = 0.0f;
  });
}

Waypoint getMissionWaypoint(int index) {
  if (index < WAYPOINT_COUNT) {
    return WAYPOINTS[index];
  }
  float holdAlt = WAYPOINT_COUNT > 0 ? WAYPOINTS[WAYPOINT_COUNT - 1].altFt : 10.0f;
  
  double lat, lon;
  withMutex([&]() { lat = sharedTrip.mission.launchLat; lon = sharedTrip.mission.launchLon; });
  return { lat, lon, holdAlt };
}

#ifdef WOKWI_SIM
void parseSimInput() {
  static String buf = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      if (buf.startsWith("HDG:")) {
        withMutex([&]() { sharedDashboard.compassHeading = buf.substring(4).toFloat(); });
      } else if (buf.startsWith("LAT:")) {
        withMutex([&]() { sharedDashboard.gps.lat = buf.substring(4).toDouble(); });
      } else if (buf.startsWith("LON:")) {
        withMutex([&]() { sharedDashboard.gps.lon = buf.substring(4).toDouble(); });
      } else if (buf.startsWith("FIX:")) {
        withMutex([&]() { sharedDashboard.gps.fix = (buf.substring(4).toInt() == 1); });
      } else if (buf.startsWith("MISSION:")) {
        // transitionTo(PHASE_MISSION) captures launch point and heading
        // itself — nothing to pre-fetch here.
        bool fixNow;
        withMutex([&]() { fixNow = sharedDashboard.gps.fix; });
        if (fixNow) {
          transitionTo(PHASE_MISSION);
        }
      }
      buf = "";
    } else if (c != '\r') {
      buf += c;
    }
  }
}
#endif

// ==========================================
// CORE 0: NAVIGATION TASK (~10Hz)
// ==========================================
void navigationTask(void* parameter) {
  const TickType_t xFrequency = pdMS_TO_TICKS(NAV_LOOP_MS);
  TickType_t lastWakeTime = xTaskGetTickCount();
  float navDt = NAV_LOOP_MS / 1000.0f; 

  for (;;) {
#ifdef WOKWI_SIM
    withMutex([&]() {
      if (sharedDashboard.gps.fix) sharedDashboard.gps.lastFixMs = millis();
      sharedDashboard.gps.sats = sharedDashboard.gps.fix ? 8 : 0;
    });
#else
    while (Serial2.available() > 0) {
      gps.encode(Serial2.read());
    }

    if (gps.location.isValid() && gps.location.age() < 2000) {
      GpsData localGps;
      localGps.lat = gps.location.lat();
      localGps.lon = gps.location.lng();
      localGps.fix = true;
      localGps.sats = gps.satellites.value();
      localGps.speedMps = gps.speed.mps();
      localGps.lastFixMs = millis();
      
      withMutex([&]() { sharedDashboard.gps = localGps; });
    } else {
      withMutex([&]() { sharedDashboard.gps.fix = false; });
    }
#endif

#ifndef WOKWI_SIM
    compass.read();
    float rawCompassHeading = compass.getAzimuth();
    withMutex([&]() { sharedDashboard.compassHeading = rawCompassHeading; });
#endif

    GpsData currentDashGps;
    RoadTripState currentTrip;
    CruiseControlState_NavData currentCruiseNav;

    withMutex([&]() {
      currentDashGps    = sharedDashboard.gps;
      currentTrip       = sharedTrip;
      currentCruiseNav  = sharedCruiseControl.nav;
    });

    // ---- Safety checks (flight timer, geofence, GPS loss) ----
    if (currentTrip.phase == PHASE_MISSION || currentTrip.phase == PHASE_HOLD) {
      unsigned long armedAt = currentArmedAtMs(currentTrip);
      if (armedAt > 0 && (millis() - armedAt) >= MAX_FLIGHT_TIME_MS) {
        logLine("[SAFETY] Max flight time reached — forcing landing.");
        stopNavigation();
        transitionTo(PHASE_LANDING);
        continue;
      }

      if (currentTrip.phase != PHASE_LANDING && currentDashGps.fix && currentTrip.mission.launchPointSet) {
        float distFromLaunch = gpsDistanceMeters(currentDashGps.lat, currentDashGps.lon,
                                                   currentTrip.mission.launchLat, currentTrip.mission.launchLon);
        if (distFromLaunch > GEOFENCE_RADIUS_M) {
          if (!currentTrip.mission.geofenceTripped) {
            withMutex([&]() { sharedTrip.mission.geofenceTripped = true; });
            logLine("[SAFETY] Geofence exceeded — aborting mission, forcing landing.");
          }
          stopNavigation();
          transitionTo(PHASE_LANDING);
          continue;
        }
      }

      if (currentTrip.phase == PHASE_MISSION && !currentDashGps.fix) {
        if (currentDashGps.lastFixMs > 0 && (millis() - currentDashGps.lastFixMs) >= GPS_LOSS_ABORT_MS) {
          if (!currentTrip.mission.gpsLossTripped) {
            withMutex([&]() { sharedTrip.mission.gpsLossTripped = true; });
            logLine("[SAFETY] GPS fix lost — aborting mission, holding/landing.");
          }
          stopNavigation();
          transitionTo(PHASE_LANDING);
          continue;
        }
      }
    }

    // ---- Waypoint guidance ----
    if (currentTrip.phase == PHASE_MISSION && currentTrip.mission.active && currentDashGps.fix) {
      if (currentTrip.mission.currentWP >= currentTrip.mission.waypointCount) {
        stopNavigation();
        transitionTo(PHASE_HOVER_SETTLE);
        logLine("[NAV] Mission complete (incl. RTL) — leveling out, hovering before landing.");
        vTaskDelayUntil(&lastWakeTime, xFrequency);
        continue;
      }

      Waypoint wp = getMissionWaypoint(currentTrip.mission.currentWP);
      float distM   = gpsDistanceMeters(currentDashGps.lat, currentDashGps.lon, wp.lat, wp.lon);
      float bearing = gpsBearing(currentDashGps.lat, currentDashGps.lon, wp.lat, wp.lon);

      withMutex([&]() { sharedCruiseControl.nav.yawTargetHeading = bearing; });

      if (distM < WAYPOINT_ACCEPT_RADIUS_M) {
        bool isRTLLeg = (currentTrip.mission.currentWP == WAYPOINT_COUNT);
        String wpReachedLog = String("[NAV] Reached ") +
                              (isRTLLeg ? String("RTL / launch point") : (String("waypoint ") + String(currentTrip.mission.currentWP))) +
                              " — advancing to index " + String(currentTrip.mission.currentWP + 1);
        logLine(wpReachedLog);

        withMutex([&]() {
          sharedTrip.mission.currentWP++;
          // Update altitude target in cruise control for the next leg.
          int nextWP = sharedTrip.mission.currentWP;
          sharedCruiseControl.nav.targetAltFt = (nextWP < sharedTrip.mission.waypointCount)
              ? getMissionWaypoint(nextWP).altFt
              : wp.altFt;
        });

        vTaskDelayUntil(&lastWakeTime, xFrequency);
        continue;
      }

      float northM, eastM;
      bearingToNorthEast(distM, bearing, northM, eastM);

      float targetPitch = navNorthPID.computeWithError(northM, navDt); 
      float targetRoll  = navEastPID.computeWithError(eastM,  navDt);  

      withMutex([&]() {
        sharedCruiseControl.nav.targetRollDeg  = targetRoll;
        sharedCruiseControl.nav.targetPitchDeg = targetPitch;
        sharedDashboard.navDistToWP            = distM;
        sharedDashboard.navBearingToWP         = bearing;
      });

    } else if (currentTrip.phase != PHASE_MISSION) {
      withMutex([&]() {
        sharedCruiseControl.nav.targetRollDeg  = 0.0f;
        sharedCruiseControl.nav.targetPitchDeg = 0.0f;
      });
    }

    // ---- Hover settle dwell ----
    if (currentTrip.phase == PHASE_HOVER_SETTLE) {
      if (millis() - currentTrip.hoverSettle.enteredAtMs >= MISSION_COMPLETE_HOVER_MS) {
        transitionTo(PHASE_LANDING);
        logLine("[NAV] Hover complete — beginning automatic landing.");
      }
    }

    // ---- Landing descent ramp ----
    // targetAltFt lives in cruise control; nav task owns ramping it down.
    if (currentTrip.phase == PHASE_LANDING) {
      bool landed = false;
      withMutex([&]() {
        float newTarget = sharedCruiseControl.nav.targetAltFt - (LAND_DESCENT_RATE_FPS * navDt);
        if (newTarget <= 0.0f) {
          sharedCruiseControl.nav.targetAltFt = 0.0f;
          landed = true;
        } else {
          sharedCruiseControl.nav.targetAltFt = newTarget;
        }
      });
      if (landed) {
        // transitionTo() takes the mutex itself — must be outside withMutex().
        transitionTo(PHASE_LANDED);
        logLine("[NAV] Landed — motors disarmed.");
      }
    }

    vTaskDelayUntil(&lastWakeTime, xFrequency);
  }
}

// ==========================================
// CORE 1: PHYSICS + FLIGHT TASK (~200Hz)
// ==========================================
unsigned long lastGyroMicros = 0;

void physicsTask(void* parameter) {
  lastGyroMicros = micros();
  filter.begin(PHYSICS_LOOP_HZ);
  const TickType_t xFrequency = pdMS_TO_TICKS(PHYSICS_LOOP_MS);
  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {
    unsigned long now = micros();
    float dt = (now - lastGyroMicros) / 1000000.0f;
    lastGyroMicros = now;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float gx = g.gyro.x * 57.2958f;
    float gy = g.gyro.y * 57.2958f;
    float gz = g.gyro.z * 57.2958f;

    if (dt > 0 && dt < 1.0f) {
      filter.updateIMU(gx, gy, gz, a.acceleration.x, a.acceleration.y, a.acceleration.z);
    }

    float currentRoll  = filter.getRoll();
    float currentPitch = filter.getPitch();
    float currentYaw   = filter.getYaw();
    float altitudeFt   = ((float)barometer.readAltitudeMeters() * 3.28084f) - groundAltitudeFt;

    CruiseControlState_NavData currentCruiseNav;
    FlightPhase currentPhase;
    float currentDashCompassHeading;

    withMutex([&]() {
      currentCruiseNav            = sharedCruiseControl.nav;
      currentPhase                = sharedTrip.phase;
      currentDashCompassHeading   = sharedDashboard.compassHeading;
    });

    CruiseControlState_MotorData cruiseMotorsOut;
    bool flightEnabled = phaseFlightEnabled(currentPhase);

    if (flightEnabled) {
      cruiseMotorsOut.baseThrottle   = altitudePID.compute(currentCruiseNav.targetAltFt, altitudeFt, dt);
      cruiseMotorsOut.rollCorrection = rollPID.compute(currentCruiseNav.targetRollDeg, currentRoll, dt);
      cruiseMotorsOut.pitchCorrection = pitchPID.compute(currentCruiseNav.targetPitchDeg, currentPitch, dt);

      float yawError = currentCruiseNav.yawTargetHeading - currentDashCompassHeading;
      if (yawError > 180.0f)  yawError -= 360.0f;
      if (yawError < -180.0f) yawError += 360.0f;
      float yawCorrection = yawPID.computeWithError(yawError, dt);

      cruiseMotorsOut.m1 = cruiseMotorsOut.baseThrottle + cruiseMotorsOut.pitchCorrection + cruiseMotorsOut.rollCorrection - yawCorrection;
      cruiseMotorsOut.m2 = cruiseMotorsOut.baseThrottle + cruiseMotorsOut.pitchCorrection - cruiseMotorsOut.rollCorrection + yawCorrection;
      cruiseMotorsOut.m3 = cruiseMotorsOut.baseThrottle - cruiseMotorsOut.pitchCorrection + cruiseMotorsOut.rollCorrection + yawCorrection;
      cruiseMotorsOut.m4 = cruiseMotorsOut.baseThrottle - cruiseMotorsOut.pitchCorrection - cruiseMotorsOut.rollCorrection - yawCorrection;

      cruiseMotorsOut.m1 = constrain(cruiseMotorsOut.m1, 0.0f, 1.0f);
      cruiseMotorsOut.m2 = constrain(cruiseMotorsOut.m2, 0.0f, 1.0f);
      cruiseMotorsOut.m3 = constrain(cruiseMotorsOut.m3, 0.0f, 1.0f);
      cruiseMotorsOut.m4 = constrain(cruiseMotorsOut.m4, 0.0f, 1.0f);

      // esc1.write(cruiseMotorsOut.m1);
      // esc2.write(cruiseMotorsOut.m2);
      // esc3.write(cruiseMotorsOut.m3);
      // esc4.write(cruiseMotorsOut.m4);
    } else {
      altitudePID.reset();
      rollPID.reset();
      pitchPID.reset();
      navNorthPID.reset();
      navEastPID.reset();
      // esc1.disarm(); esc2.disarm(); esc3.disarm(); esc4.disarm();
    }

    ImuData dashImuOut;
    dashImuOut.roll  = currentRoll;
    dashImuOut.pitch = currentPitch;
    dashImuOut.yaw   = currentYaw;
    dashImuOut.accX  = a.acceleration.x;
    dashImuOut.accY  = a.acceleration.y;
    dashImuOut.accZ  = a.acceleration.z;
    dashImuOut.temp  = temp.temperature;
    dashImuOut.altitudeFt = altitudeFt;

    withMutex([&]() {
      sharedDashboard.imu           = dashImuOut;
      sharedCruiseControl.motors    = cruiseMotorsOut;
    });

    vTaskDelayUntil(&lastWakeTime, xFrequency);
  }
}

// ==========================================
// WEB FORMATTERS
// ==========================================
String getGyroReadings() {
  ImuData dashImu;
  withMutex([&]() { dashImu = sharedDashboard.imu; });
  
  readings["gyroX"] = dashImu.roll;
  readings["gyroY"] = dashImu.pitch;
  readings["gyroZ"] = dashImu.yaw;
  String out; serializeJson(readings, out); return out;
}

String getAccReadings() {
  ImuData dashImu;
  withMutex([&]() { dashImu = sharedDashboard.imu; });
  
  readings["accX"] = dashImu.accX; 
  readings["accY"] = dashImu.accY; 
  readings["accZ"] = dashImu.accZ;
  String out; serializeJson(readings, out); return out;
}

String getFlightReadings() {
  ImuData dashImu;
  CruiseControlState_MotorData cruiseMotors;
  GpsData dashGps;
  CruiseControlState_NavData cruiseNav;
  RoadTripState trip;
  float dashCompassHeading;

  float navDistToWP, navBearingToWP;
  withMutex([&]() {
    dashImu            = sharedDashboard.imu;
    cruiseMotors       = sharedCruiseControl.motors;
    dashGps            = sharedDashboard.gps;
    cruiseNav          = sharedCruiseControl.nav;
    trip               = sharedTrip;
    dashCompassHeading = sharedDashboard.compassHeading;
    navDistToWP        = sharedDashboard.navDistToWP;
    navBearingToWP     = sharedDashboard.navBearingToWP;
  });

  readings["altFt"]           = dashImu.altitudeFt;
  readings["targetFt"]        = cruiseNav.targetAltFt;
  readings["m1"]              = (int)(cruiseMotors.m1 * 100);
  readings["m2"]              = (int)(cruiseMotors.m2 * 100);
  readings["m3"]              = (int)(cruiseMotors.m3 * 100);
  readings["m4"]              = (int)(cruiseMotors.m4 * 100);
  readings["baseThrottle"]    = (int)(cruiseMotors.baseThrottle   * 100);
  readings["rollCorrection"]  = (int)(cruiseMotors.rollCorrection  * 100);
  readings["pitchCorrection"] = (int)(cruiseMotors.pitchCorrection * 100);
  readings["flightEnabled"]   = phaseFlightEnabled(trip.phase);
  readings["gpsFix"]          = dashGps.fix;
  readings["gpsLat"]          = dashGps.lat;
  readings["gpsLon"]          = dashGps.lon;
  readings["gpsSats"]         = dashGps.sats;
  readings["compassHeading"]  = dashCompassHeading;
  readings["navActive"]       = (trip.phase == PHASE_MISSION) && trip.mission.active;
  readings["navWaypoint"]     = trip.mission.currentWP;
  readings["navWaypointCount"]= trip.mission.waypointCount;
  readings["navDistM"]        = navDistToWP;
  readings["navBearing"]      = navBearingToWP;
  readings["flightPhase"]     = phaseName(trip.phase);

  bool hasArmedClock = (trip.phase == PHASE_HOLD || trip.phase == PHASE_MISSION);
  readings["flightSecRemaining"] = hasArmedClock
      ? max(0L, (long)((MAX_FLIGHT_TIME_MS - (millis() - currentArmedAtMs(trip))) / 1000))
      : (long)(MAX_FLIGHT_TIME_MS / 1000);

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
  else logLine("[COMPASS] WARNING: no saved calibration found.");

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
    int16_t x = compass.getX();
    int16_t y = compass.getY();
    int16_t z = compass.getZ();

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
  float avgRange = (rangeX + rangeY + rangeZ) / 3.0f;
  
  compassScaleX = (rangeX > 0) ? (avgRange / rangeX) : 1.0f;
  compassScaleY = (rangeY > 0) ? (avgRange / rangeY) : 1.0f;
  compassScaleZ = (rangeZ > 0) ? (avgRange / rangeZ) : 1.0f;

  compass.setCalibration(compassOffsetX, compassOffsetY, compassOffsetZ,
                          compassScaleX,  compassScaleY,  compassScaleZ);

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

  if (CALIBRATE_COMPASS_ON_BOOT) runCompassCalibration();
  else loadCompassCalibration();

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
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }
  logLine(String("[WiFi] ") + WiFi.localIP().toString());
}

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
  // esc1.init(4, RMT_CHANNEL_0);
  // esc2.init(5, RMT_CHANNEL_1); 
  // esc3.init(6, RMT_CHANNEL_2); 
  // esc4.init(7, RMT_CHANNEL_3); 
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
    withMutex([&]() { hasFix = sharedDashboard.gps.fix; });

    if (!hasFix) {
      r->send(400, "text/plain", "NO GPS FIX — wait for satellite lock before starting mission");
      return;
    }

    transitionTo(PHASE_MISSION);
    logLine("[FLIGHT] Drone Armed & Mission Started. Launch point captured.");
    r->send(200, "text/plain", "WAYPOINT NAVIGATION STARTED");
  });

  server.on("/abort", HTTP_GET, [](AsyncWebServerRequest* r) {
    stopNavigation();
    FlightPhase currentPhase;
    withMutex([&]() { currentPhase = sharedTrip.phase; });
    if (currentPhase == PHASE_MISSION) {
      transitionTo(PHASE_HOLD);
    }
    logLine("[MISSION] Mission aborted — holding position.");
    r->send(200, "text/plain", "MISSION ABORTED — HOLDING");
  });

  server.on("/land", HTTP_GET, [](AsyncWebServerRequest* r) {
    stopNavigation();
    transitionTo(PHASE_LANDING);
    logLine("[FLIGHT] Manual landing requested.");
    r->send(200, "text/plain", "LANDING");
  });

  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest* r) {
    stopNavigation();
    transitionTo(PHASE_IDLE);
    logLine("[FLIGHT] EMERGENCY STOP — motors cut.");
    r->send(200, "text/plain", "STOPPED");
  });

  server.on("/calibrate-compass", HTTP_GET, [](AsyncWebServerRequest* r) {
    bool enabled;
    withMutex([&]() { enabled = phaseFlightEnabled(sharedTrip.phase); });
    
    if (enabled) {
      r->send(400, "text/plain", "REFUSED — disarm first");
      return;
    }
    r->send(200, "text/plain", "CALIBRATING — rotate the drone slowly for 30 seconds");
    runCompassCalibration();
  });

  events.onConnect([](AsyncEventSourceClient* c) {
    c->send("hello!", NULL, millis(), 10000);
  });
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