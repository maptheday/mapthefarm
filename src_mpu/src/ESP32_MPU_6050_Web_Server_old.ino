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
// DATA STRUCTURES — SENSORS / ACTUATORS
// These exist continuously, independent of flight phase.
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
    case PHASE_HOLD:        return "HOLD";
    case PHASE_MISSION:      return "MISSION";
    case PHASE_HOVER_SETTLE: return "HOVER_SETTLE";
    case PHASE_LANDING:      return "LANDING";
    case PHASE_LANDED:       return "LANDED";
  }
  return "UNKNOWN";
}

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

struct MotorData {
  float m1 = 0, m2 = 0, m3 = 0, m4 = 0;
  float baseThrottle = 0;
  float rollCorrection = 0;
  float pitchCorrection = 0;
};

// Roll/pitch/yaw targets are read by physicsTask on every 200Hz tick
// regardless of flight phase (e.g. yaw heading is deliberately held
// through HOVER_SETTLE/LANDING, not reset) — so this stays continuous,
// shared state rather than living inside a phase object.
struct NavData {
  float targetRollDeg = 0.0f;
  float targetPitchDeg = 0.0f;
  float distToWP = 0.0f;
  float bearingToWP = 0.0f;
  float yawTargetHeading = 0.0f;
};

// Raw/fused sensor inputs — what the hardware is physically telling us
// right now. Nothing here is computed or commanded, only measured.
struct SensorState {
  GpsData gps;
  ImuData imu;
  float compassHeading = 0.0f;
};

// PID-computed guidance targets and the resulting actuator commands —
// not sensed, but just as continuous and phase-independent as the
// sensor readings above, which is why this still lives outside
// SystemState rather than inside any one phase's object.
struct ControlState {
  NavData nav;
  MotorData motors;
};

// ==========================================
// DATA STRUCTURES — PER-PHASE STATE
//
// Every field the system tracks while flying lives inside the object
// for the phase it belongs to. Nothing about "what's true during
// MISSION" is split off into a shared top-level field, or set up in a
// separate step before the phase transition — it's all right here in
// MissionState, and transitionTo() populates all of it in one shot.
// Only the struct matching the CURRENT phase is meaningful; the others
// hold stale data from the last time that phase ran.
//
// currentFlightEnabled() / currentTargetAltFt() / currentArmedAtMs()
// below are the one place that knows how to pick the right struct —
// nothing else in the file should reach into e.g. sharedSys.landing.*
// without going through sharedSys.phase first.
// ==========================================
struct IdleState {
  // Disarmed, motors off. Nothing to track.
};

struct HoldState {
  unsigned long armedAtMs = 0;   // start of the max-flight-time safety clock
  bool flightEnabled = true;
  float targetAltFt = 10.0f;     // holding this altitude
};

struct MissionState {
  unsigned long armedAtMs = 0;
  bool flightEnabled = true;
  float targetAltFt = 10.0f;
  int currentWP = 0;
  int waypointCount = 0;
  bool geofenceTripped = false;
  bool gpsLossTripped = false;
  bool active = false;           // horizontal waypoint guidance running
  double launchLat = 0.0;        // captured from GPS the instant MISSION starts
  double launchLon = 0.0;
  bool launchPointSet = false;
};

struct HoverSettleState {
  unsigned long enteredAtMs = 0;
  bool flightEnabled = true;
  float targetAltFt = 10.0f;     // holding the RTL altitude before landing
};

struct LandingState {
  bool flightEnabled = true;
  float targetAltFt = 10.0f;     // ramps down toward 0 every nav tick
};

struct LandedState {
  bool flightEnabled = false;
  float targetAltFt = 0.0f;
};

struct SystemState {
  FlightPhase phase = PHASE_IDLE;
  IdleState idle;
  HoldState hold;
  MissionState mission;
  HoverSettleState hoverSettle;
  LandingState landing;
  LandedState landed;
};

// Single lookup point for "what's the live value of X right now" —
// picks the field out of whichever phase struct is actually active.
bool currentFlightEnabled(const SystemState& sys) {
  switch (sys.phase) {
    case PHASE_IDLE:         return false; // IdleState has no such field — always disarmed
    case PHASE_HOLD:        return sys.hold.flightEnabled;
    case PHASE_MISSION:      return sys.mission.flightEnabled;
    case PHASE_HOVER_SETTLE: return sys.hoverSettle.flightEnabled;
    case PHASE_LANDING:      return sys.landing.flightEnabled;
    case PHASE_LANDED:       return sys.landed.flightEnabled;
  }
  return false;
}

float currentTargetAltFt(const SystemState& sys) {
  switch (sys.phase) {
    case PHASE_IDLE:         return 0.0f;
    case PHASE_HOLD:        return sys.hold.targetAltFt;
    case PHASE_MISSION:      return sys.mission.targetAltFt;
    case PHASE_HOVER_SETTLE: return sys.hoverSettle.targetAltFt;
    case PHASE_LANDING:      return sys.landing.targetAltFt;
    case PHASE_LANDED:       return sys.landed.targetAltFt;
  }
  return 0.0f;
}

unsigned long currentArmedAtMs(const SystemState& sys) {
  switch (sys.phase) {
    case PHASE_HOLD:   return sys.hold.armedAtMs;
    case PHASE_MISSION: return sys.mission.armedAtMs;
    default:             return 0;
  }
}

// ==========================================
// SHARED STATE VARIABLES
// ==========================================
SemaphoreHandle_t sharedDataMutex;
SemaphoreHandle_t serialMutex;  

volatile SensorState  sharedSensors;
volatile ControlState sharedControl;
volatile SystemState  sharedSys;

// One-time calibration baseline measured at boot, not a live reading —
// stays a plain global rather than living in ActiveSensorState.
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
// One function owns every jump between flight phases, INCLUDING
// capturing anything the incoming phase needs from live sensor state
// (e.g. MISSION grabs the launch point straight from sharedSensors.gps here —
// nothing needs to be pre-fetched by the caller and handed in).
// Whatever needs to carry over from the outgoing phase into the
// incoming one — the safety clock surviving MISSION -> HOLD on abort,
// the landing ramp starting from wherever the drone actually was —
// also happens here, explicitly, instead of being duplicated at every
// callsite.
//
// NOTE: takes the mutex itself — never call this from inside another
// withMutex(...) lambda, sharedDataMutex is not recursive.
// ==========================================
void transitionTo(FlightPhase next) {
  withMutex([&]() {
    switch (next) {
      case PHASE_HOLD:
        // Only reached via /abort (MISSION -> HOLD). Carry the safety
        // clock and current altitude target forward so aborting doesn't
        // reset the flight timer or make the drone jump altitude.
        sharedSys.hold.armedAtMs   = sharedSys.mission.armedAtMs;
        sharedSys.hold.flightEnabled = true;
        sharedSys.hold.targetAltFt = sharedSys.mission.targetAltFt;
        break;

      case PHASE_MISSION:
        sharedSys.mission.armedAtMs = millis();
        sharedSys.mission.flightEnabled = true;
        sharedSys.mission.targetAltFt = WAYPOINTS[0].altFt;
        sharedSys.mission.currentWP = 0;
        sharedSys.mission.waypointCount = WAYPOINT_COUNT + 1; // +1 for the appended RTL leg
        sharedSys.mission.geofenceTripped = false;
        sharedSys.mission.gpsLossTripped = false;
        sharedSys.mission.active = true;
        // Launch point + starting heading, captured straight off the
        // current sensor state — this IS what "starting a mission" means.
        sharedSys.mission.launchLat = sharedSensors.gps.lat;
        sharedSys.mission.launchLon = sharedSensors.gps.lon;
        sharedSys.mission.launchPointSet = true;
        sharedControl.nav.yawTargetHeading = sharedSensors.compassHeading;
        break;

      case PHASE_HOVER_SETTLE:
        sharedSys.hoverSettle.enteredAtMs = millis();
        sharedSys.hoverSettle.flightEnabled = true;
        sharedSys.hoverSettle.targetAltFt = sharedSys.mission.targetAltFt; // hold the RTL altitude
        break;

      case PHASE_LANDING:
        sharedSys.landing.flightEnabled = true;
        // Start the descent ramp from wherever the previous phase left
        // the altitude target — read BEFORE sharedSys.phase flips below.
        sharedSys.landing.targetAltFt = currentTargetAltFt(sharedSys);
        break;

      case PHASE_LANDED:
        sharedSys.landed.flightEnabled = false;
        sharedSys.landed.targetAltFt = 0.0f;
        break;

      case PHASE_IDLE:
        // IdleState has no fields — currentFlightEnabled()/currentTargetAltFt()
        // hardcode false/0 for this phase.
        break;
    }
    sharedSys.phase = next;
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
    sharedSys.mission.active = false;
    sharedControl.nav.targetRollDeg = 0.0f;
    sharedControl.nav.targetPitchDeg = 0.0f;
  });
}

Waypoint getMissionWaypoint(int index) {
  if (index < WAYPOINT_COUNT) {
    return WAYPOINTS[index];
  }
  float holdAlt = WAYPOINT_COUNT > 0 ? WAYPOINTS[WAYPOINT_COUNT - 1].altFt : 10.0f;
  
  double lat, lon;
  withMutex([&]() { lat = sharedSys.mission.launchLat; lon = sharedSys.mission.launchLon; });
  return { lat, lon, holdAlt };
}

#ifdef WOKWI_SIM
void parseSimInput() {
  static String buf = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      if (buf.startsWith("HDG:")) {
        withMutex([&]() { sharedSensors.compassHeading = buf.substring(4).toFloat(); });
      } else if (buf.startsWith("LAT:")) {
        withMutex([&]() { sharedSensors.gps.lat = buf.substring(4).toDouble(); });
      } else if (buf.startsWith("LON:")) {
        withMutex([&]() { sharedSensors.gps.lon = buf.substring(4).toDouble(); });
      } else if (buf.startsWith("FIX:")) {
        withMutex([&]() { sharedSensors.gps.fix = (buf.substring(4).toInt() == 1); });
      } else if (buf.startsWith("MISSION:")) {
        // Sim triggering mission directly. transitionTo() grabs the
        // launch point/heading itself — nothing to prep here but the
        // fix check.
        bool fixNow;
        withMutex([&]() { fixNow = sharedSensors.gps.fix; });
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
      if (sharedSensors.gps.fix) sharedSensors.gps.lastFixMs = millis();
      sharedSensors.gps.sats = sharedSensors.gps.fix ? 8 : 0;
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
      
      withMutex([&]() { sharedSensors.gps = localGps; });
    } else {
      withMutex([&]() { sharedSensors.gps.fix = false; });
    }
#endif

#ifndef WOKWI_SIM
    compass.read();
    float heading = compass.getAzimuth();
    withMutex([&]() { sharedSensors.compassHeading = heading; });
#endif

    GpsData currentGps;
    SystemState currentSys;
    NavData currentNav;

    withMutex([&]() {
      currentGps = sharedSensors.gps;
      currentSys = sharedSys;
      currentNav = sharedControl.nav;
    });

    if (currentSys.phase == PHASE_MISSION || currentSys.phase == PHASE_HOLD) {
      unsigned long armedAt = currentArmedAtMs(currentSys);
      if (armedAt > 0 && (millis() - armedAt) >= MAX_FLIGHT_TIME_MS) {
        logLine("[SAFETY] Max flight time reached — forcing landing.");
        stopNavigation();
        transitionTo(PHASE_LANDING);
        continue;
      }

      if (currentSys.phase != PHASE_LANDING && currentGps.fix && currentSys.mission.launchPointSet) {
        float distFromLaunch = gpsDistanceMeters(currentGps.lat, currentGps.lon,
                                                   currentSys.mission.launchLat, currentSys.mission.launchLon);
        if (distFromLaunch > GEOFENCE_RADIUS_M) {
          if (!currentSys.mission.geofenceTripped) {
            withMutex([&]() { sharedSys.mission.geofenceTripped = true; });
            logLine("[SAFETY] Geofence exceeded — aborting mission, forcing landing.");
          }
          stopNavigation();
          transitionTo(PHASE_LANDING);
          continue;
        }
      }

      if (currentSys.phase == PHASE_MISSION && !currentGps.fix) {
        if (currentGps.lastFixMs > 0 && (millis() - currentGps.lastFixMs) >= GPS_LOSS_ABORT_MS) {
          if (!currentSys.mission.gpsLossTripped) {
            withMutex([&]() { sharedSys.mission.gpsLossTripped = true; });
            logLine("[SAFETY] GPS fix lost — aborting mission, holding/landing.");
          }
          stopNavigation();
          transitionTo(PHASE_LANDING);
          continue;
        }
      }
    }

    if (currentSys.phase == PHASE_MISSION && currentSys.mission.active && currentGps.fix) {
      if (currentSys.mission.currentWP >= currentSys.mission.waypointCount) {
        stopNavigation();
        transitionTo(PHASE_HOVER_SETTLE);
        logLine("[NAV] Mission complete (incl. RTL) — leveling out, hovering before landing.");
        vTaskDelayUntil(&lastWakeTime, xFrequency);
        continue;
      }

      Waypoint wp = getMissionWaypoint(currentSys.mission.currentWP);
      float distM   = gpsDistanceMeters(currentGps.lat, currentGps.lon, wp.lat, wp.lon);
      float bearing = gpsBearing(currentGps.lat, currentGps.lon, wp.lat, wp.lon);

      withMutex([&]() { sharedControl.nav.yawTargetHeading = bearing; });

      if (distM < WAYPOINT_ACCEPT_RADIUS_M) {
        bool isRTLLeg = (currentSys.mission.currentWP == WAYPOINT_COUNT);
        String wpReachedLog = String("[NAV] Reached ") +
                              (isRTLLeg ? String("RTL / launch point") : (String("waypoint ") + String(currentSys.mission.currentWP))) +
                              " — advancing to index " + String(currentSys.mission.currentWP + 1);
        logLine(wpReachedLog);

        withMutex([&]() {
          sharedSys.mission.currentWP++;
          sharedSys.mission.targetAltFt = (sharedSys.mission.currentWP < sharedSys.mission.waypointCount)
                                          ? getMissionWaypoint(sharedSys.mission.currentWP).altFt
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
        sharedControl.nav.targetRollDeg  = targetRoll;
        sharedControl.nav.targetPitchDeg = targetPitch;
        sharedControl.nav.distToWP       = distM;
        sharedControl.nav.bearingToWP    = bearing;
      });

    } else if (currentSys.phase != PHASE_MISSION) {
      withMutex([&]() {
        sharedControl.nav.targetRollDeg  = 0.0f;
        sharedControl.nav.targetPitchDeg = 0.0f;
      });
    }

    if (currentSys.phase == PHASE_HOVER_SETTLE) {
      if (millis() - currentSys.hoverSettle.enteredAtMs >= MISSION_COMPLETE_HOVER_MS) {
        transitionTo(PHASE_LANDING);
        logLine("[NAV] Hover complete — beginning automatic landing.");
      }
    }

    if (currentSys.phase == PHASE_LANDING) {
      bool landed = false;
      withMutex([&]() {
        float newTarget = sharedSys.landing.targetAltFt - (LAND_DESCENT_RATE_FPS * navDt);
        if (newTarget <= 0.0f) {
          sharedSys.landing.targetAltFt = 0.0f;
          landed = true;
        } else {
          sharedSys.landing.targetAltFt = newTarget;
        }
      });
      if (landed) {
        // transitionTo() takes the mutex itself, so it has to happen
        // outside the withMutex(...) block above — the lock isn't recursive.
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
    float altitudeFt = ((float)barometer.readAltitudeMeters() * 3.28084f) - groundAltitudeFt;

    NavData currentNav;
    SystemState currentSys;
    float currentCompassHeading;

    withMutex([&]() {
      currentNav = sharedControl.nav;
      currentSys = sharedSys;
      currentCompassHeading = sharedSensors.compassHeading;
    });

    MotorData localMotors;
    bool flightEnabled = currentFlightEnabled(currentSys);

    if (flightEnabled) {
      localMotors.baseThrottle = altitudePID.compute(currentTargetAltFt(currentSys), altitudeFt, dt);
      localMotors.rollCorrection = rollPID.compute(currentNav.targetRollDeg, currentRoll, dt);
      localMotors.pitchCorrection = pitchPID.compute(currentNav.targetPitchDeg, currentPitch, dt);

      float yawError = currentNav.yawTargetHeading - currentCompassHeading;
      if (yawError > 180.0f)  yawError -= 360.0f;
      if (yawError < -180.0f) yawError += 360.0f;
      float yawCorrection = yawPID.computeWithError(yawError, dt);

      localMotors.m1 = localMotors.baseThrottle + localMotors.pitchCorrection + localMotors.rollCorrection - yawCorrection;
      localMotors.m2 = localMotors.baseThrottle + localMotors.pitchCorrection - localMotors.rollCorrection + yawCorrection;
      localMotors.m3 = localMotors.baseThrottle - localMotors.pitchCorrection + localMotors.rollCorrection + yawCorrection;
      localMotors.m4 = localMotors.baseThrottle - localMotors.pitchCorrection - localMotors.rollCorrection - yawCorrection;

      localMotors.m1 = constrain(localMotors.m1, 0.0f, 1.0f);
      localMotors.m2 = constrain(localMotors.m2, 0.0f, 1.0f);
      localMotors.m3 = constrain(localMotors.m3, 0.0f, 1.0f);
      localMotors.m4 = constrain(localMotors.m4, 0.0f, 1.0f);

      // esc1.write(localMotors.m1);
      // esc2.write(localMotors.m2);
      // esc3.write(localMotors.m3);
      // esc4.write(localMotors.m4);
    } else {
      altitudePID.reset();
      rollPID.reset();
      pitchPID.reset();
      navNorthPID.reset();
      navEastPID.reset();
      // esc1.disarm(); esc2.disarm(); esc3.disarm(); esc4.disarm();
    }

    ImuData localImu;
    localImu.roll = currentRoll;
    localImu.pitch = currentPitch;
    localImu.yaw = currentYaw;
    localImu.accX = a.acceleration.x;
    localImu.accY = a.acceleration.y;
    localImu.accZ = a.acceleration.z;
    localImu.temp = temp.temperature;
    localImu.altitudeFt = altitudeFt;

    withMutex([&]() {
      sharedSensors.imu = localImu;
      sharedControl.motors = localMotors;
    });

    vTaskDelayUntil(&lastWakeTime, xFrequency);
  }
}

// ==========================================
// WEB FORMATTERS
// ==========================================
String getGyroReadings() {
  ImuData imu;
  withMutex([&]() { imu = sharedSensors.imu; });
  
  readings["gyroX"] = imu.roll;
  readings["gyroY"] = imu.pitch;
  readings["gyroZ"] = imu.yaw;
  String out; serializeJson(readings, out); return out;
}

String getAccReadings() {
  ImuData imu;
  withMutex([&]() { imu = sharedSensors.imu; });
  
  readings["accX"] = imu.accX; 
  readings["accY"] = imu.accY; 
  readings["accZ"] = imu.accZ;
  String out; serializeJson(readings, out); return out;
}

String getFlightReadings() {
  ImuData imu;
  MotorData motors;
  GpsData gpsLocal;
  NavData nav;
  SystemState sys;
  float heading;

  withMutex([&]() {
    imu = sharedSensors.imu;
    motors = sharedControl.motors;
    gpsLocal = sharedSensors.gps;
    nav = sharedControl.nav;
    sys = sharedSys;
    heading = sharedSensors.compassHeading;
  });

  readings["altFt"]           = imu.altitudeFt;
  readings["targetFt"]        = currentTargetAltFt(sys);
  readings["m1"]              = (int)(motors.m1 * 100);
  readings["m2"]              = (int)(motors.m2 * 100);
  readings["m3"]              = (int)(motors.m3 * 100);
  readings["m4"]              = (int)(motors.m4 * 100);
  readings["baseThrottle"]    = (int)(motors.baseThrottle  * 100);
  readings["rollCorrection"]  = (int)(motors.rollCorrection  * 100);
  readings["pitchCorrection"] = (int)(motors.pitchCorrection * 100);
  readings["flightEnabled"]   = currentFlightEnabled(sys);
  readings["gpsFix"]          = gpsLocal.fix;
  readings["gpsLat"]          = gpsLocal.lat;
  readings["gpsLon"]          = gpsLocal.lon;
  readings["gpsSats"]         = gpsLocal.sats;
  readings["compassHeading"]  = heading;
  readings["navActive"]       = (sys.phase == PHASE_MISSION) && sys.mission.active;
  readings["navWaypoint"]     = sys.mission.currentWP;
  readings["navWaypointCount"]= sys.mission.waypointCount;
  readings["navDistM"]        = nav.distToWP;
  readings["navBearing"]      = nav.bearingToWP;
  readings["flightPhase"]     = phaseName(sys.phase);

  unsigned long armedAt = currentArmedAtMs(sys);
  readings["flightSecRemaining"] = armedAt > 0
      ? max(0L, (long)((MAX_FLIGHT_TIME_MS - (millis() - armedAt)) / 1000))
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

  // ==========================================
  // SINGLE FIRE-AND-FORGET FLIGHT ENDPOINT
  // Combines previous /fly and /mission actions
  // ==========================================
  server.on("/start-waypoint-nav", HTTP_GET, [](AsyncWebServerRequest* r) {
    bool hasFix;
    withMutex([&]() { hasFix = sharedSensors.gps.fix; });

    if (!hasFix) {
      r->send(400, "text/plain", "NO GPS FIX — wait for satellite lock before starting mission");
      return;
    }

    // transitionTo(PHASE_MISSION) captures the launch point and current
    // heading itself, straight from sharedSensors, and sets
    // up the rest of MissionState — nothing to pre-fetch or set up here.
    transitionTo(PHASE_MISSION);

    logLine("[FLIGHT] Drone Armed & Mission Started. Launch point captured.");
    r->send(200, "text/plain", "WAYPOINT NAVIGATION STARTED");
  });

  // Keep these strictly for testing or emergency API overrides
  server.on("/abort", HTTP_GET, [](AsyncWebServerRequest* r) {
    stopNavigation();
    FlightPhase currentPhase;
    withMutex([&]() { currentPhase = sharedSys.phase; });
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
    withMutex([&]() { enabled = currentFlightEnabled(sharedSys); });
    
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