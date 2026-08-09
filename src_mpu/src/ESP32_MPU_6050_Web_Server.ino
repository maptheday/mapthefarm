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

#include "EspBarometer.hpp"
#include "EspESC.hpp"
#include "PID.hpp"

const char* ssid     = "Wokwi-GUEST";
const char* password = "";

// ==========================================
// TIMING CONFIGURATION
// Increase these values to slow down the loops.
// ==========================================
// change this back to 5ms for 200Hz physics loop once testing is done
const unsigned long PHYSICS_LOOP_MS = 5;
const float PHYSICS_LOOP_HZ = 1000.0f / PHYSICS_LOOP_MS;
const unsigned long SSE_GYRO_MS    = 10;  // orientation updates
const unsigned long SSE_ACC_MS     = 200; // accelerometer updates
const unsigned long SSE_FLIGHT_MS  = 100; // flight/state updates
const unsigned long NAV_LOOP_MS    = 100; // navigation updates at 10Hz
                                          // GPS only updates at 1-10Hz anyway,
                                          // no point running nav faster than that

// ==========================================
// BN-880 WIRING
//
// The BN-880 is two devices in one module:
//   GPS chip  — talks over UART (serial text, NMEA sentences)
//   QMC5883L compass chip — talks over I2C (same bus as MPU6050)
//
// GPS UART:
//   BN-880 TX → ESP32-S3 GPIO 17 (Serial2 RX)
//   BN-880 RX → ESP32-S3 GPIO 18 (Serial2 TX)
//   Baud rate: 9600 (BN-880 default)
//
// Compass I2C (shared bus with MPU6050 and BME280):
//   BN-880 SDA → GPIO 16
//   BN-880 SCL → GPIO 15
//   I2C address: 0x0D (QMC5883L default)
//
// Power:
//   BN-880 VCC → 3.3V
//   BN-880 GND → GND
// ==========================================
#define GPS_RX_PIN 17
#define GPS_TX_PIN 18
#define GPS_BAUD   9600

// ==========================================
// WAYPOINT SYSTEM
//
// A waypoint is a GPS coordinate the drone flies toward.
// For Map The Farm, these are points along a fence line.
//
// The drone flies from waypoint to waypoint in order.
// When it reaches the last waypoint, the mission is complete
// and it holds position until told to land.
//
// To load a fence line mission, populate the WAYPOINTS array
// with your GPS coordinates from Google Maps or a survey.
// ==========================================
struct Waypoint {
  double lat;       // latitude in decimal degrees  (e.g. 36.123456)
  double lon;       // longitude in decimal degrees (e.g. -80.123456)
  float  altFt;     // target altitude in feet above ground
};

// ── Fence line waypoints — replace with real GPS coords ──────────────
// How to get these: stand at each fence corner with your phone,
// open Google Maps, long-press to drop a pin, read the lat/lon.
// The drone will fly these in order: wp[0] → wp[1] → wp[2] → ...
const Waypoint WAYPOINTS[] = {
  { 36.123456, -80.123456, 30.0f },  // fence corner A
  { 36.123789, -80.123456, 30.0f },  // fence corner B
  { 36.123789, -80.123789, 30.0f },  // fence corner C
  { 36.123456, -80.123789, 30.0f },  // fence corner D (back to start)
};
const int WAYPOINT_COUNT = sizeof(WAYPOINTS) / sizeof(WAYPOINTS[0]);

// How close (in meters) the drone needs to get to a waypoint
// before it advances to the next one.
// 2.0m is tight but achievable with good GPS fix.
// Increase to 5.0f if it overshoots.
const float WAYPOINT_ACCEPT_RADIUS_M = 5.0f;

// ==========================================
// WOKWI SIMULATION STATE
// Only exists in the wokwi_sim build. Populated by parseSimInput()
// reading commands sent over Serial from the scenario YAML's write-serial steps.
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
TinyGPSPlus gps;           // parses NMEA sentences from BN-880 GPS
QMC5883LCompass compass;   // reads heading from BN-880 QMC5883L chip

AsyncWebServer server(80);          // ← add this
AsyncEventSource events("/events"); // ← add this

// One ESC instance per motor (GPIO 4/5/6/7 → RMT channels 0/1/2/3)
// EspESC esc1, esc2, esc3, esc4;

// ==========================================
// PID CONTROLLERS
//
// Altitude PID:
//   error   = target altitude - actual altitude (feet)
//   output  = base throttle for all 4 motors (0.0 - 1.0)
//   Kp=0.08  — gentle push per foot of error
//   Ki=0.01  — slowly corrects persistent hover drift
//   Kd=0.05  — slows down as we approach target altitude
//
// Roll PID:
//   error   = 0° - actual roll (degrees)
//   output  = left/right motor trim (-0.3 to +0.3)
//   Kp=0.01  — small correction per degree of tilt
//   Ki=0.001 — corrects slow sideways drift
//   Kd=0.005 — dampens oscillation
//
// Pitch PID:
//   Same gains as roll, front/rear motor trim
//
// Yaw PID:
//   For yaw, the correction is gentler than roll/pitch because
//   spinning the whole drone is slower and more sluggish than tilting it.
//   Now uses real compass heading instead of gyro yaw — no drift.
//
// Navigation PIDs:
//   Two new PIDs that sit ABOVE the roll/pitch PIDs.
//   They take GPS position error (how far from the waypoint)
//   and convert it into a target tilt angle for roll/pitch to chase.
//
//   Think of it as nested loops:
//     Navigation (10Hz): "I need to move north 10m" → tilt nose forward 5°
//     Pitch PID (200Hz): "nose is at 0°, target is 5°" → spin rear motors
//
//   navNorthPID  — controls forward/backward movement (pitch axis)
//   navEastPID   — controls left/right movement (roll axis)
//
//   Output is in degrees of tilt (-15° to +15°).
//   Small Kp because GPS is noisy — aggressive gains cause oscillation.
// ==========================================
PID altitudePID (0.08f,  0.01f,  0.05f,  0.0f,   1.0f );
PID rollPID     (0.01f,  0.001f, 0.005f, -0.3f,  0.3f );
PID pitchPID    (0.01f,  0.001f, 0.005f, -0.3f,  0.3f );
PID yawPID      (0.005f, 0.0001f,0.001f, -0.2f,  0.2f );
PID navNorthPID (0.5f,   0.0f,   0.1f,   -15.0f, 15.0f); // output = target pitch degrees
PID navEastPID  (0.5f,   0.0f,   0.1f,   -15.0f, 15.0f); // output = target roll degrees

// ==========================================
// SHARED STATE
// Written by one core, read by the other.
// Always use sharedDataMutex before touching these.
// ==========================================
SemaphoreHandle_t sharedDataMutex;
SemaphoreHandle_t serialMutex;  // protects Serial.print* calls from interleaving

// IMU (written by Core 1 physics task)
float sharedRoll  = 0, sharedPitch = 0, sharedYaw = 0;
float sharedAccX  = 0, sharedAccY  = 0, sharedAccZ = 0;
float sharedTemp  = 0;

// Barometer (written by Core 1 physics task)
float groundAltitudeFt = 0; // measured at startup, used to compute relative altitude
float sharedAltitudeFt = 0;

// Motors (written by Core 1, sent as 0-100 to UI)
float sharedM1 = 0, sharedM2 = 0, sharedM3 = 0, sharedM4 = 0;

// PID debug (written by Core 1, read by web formatter)
float sharedBaseThrottle    = 0;
float sharedRollCorrection  = 0;
float sharedPitchCorrection = 0;

// GPS (written by Core 0 nav task, read by Core 1 for logging/UI)
volatile double gpsLat       = 0.0;   // current latitude
volatile double gpsLon       = 0.0;   // current longitude
volatile bool   gpsFix       = false; // true = GPS has satellite lock
volatile int    gpsSats      = 0;     // number of satellites in view
volatile float  gpsSpeedMps  = 0.0f;  // ground speed in m/s

// Compass (written by Core 0 nav task, read by Core 1 physics task for yaw PID)
// This replaces the gyro-based yaw which drifts over time.
// The compass always knows true heading regardless of how long we've been flying.
volatile float compassHeading = 0.0f; // 0-360 degrees, 0 = north

// Navigation (written by Core 0 nav task, read by Core 1 physics task)
// When navActive is true, the physics task uses these target angles
// instead of 0.0f for roll and pitch — the drone tilts toward the waypoint.
volatile bool  navActive        = false; // true = flying a waypoint mission
volatile float navTargetRollDeg = 0.0f;  // target roll angle from nav PID
volatile float navTargetPitchDeg= 0.0f;  // target pitch angle from nav PID
volatile int   navCurrentWP     = 0;     // which waypoint we're flying toward
volatile float navDistToWP      = 0.0f;  // meters to current waypoint (for UI)
volatile float navBearingToWP   = 0.0f;  // degrees to current waypoint (for UI)

// Flags / commands (written by Core 0 web handlers, consumed by Core 1)
volatile bool  flightEnabled = true;
volatile float targetAltFt   = 10.0f;

// for the endpoints
JsonDocument readings;

// ==========================================
// GPS / NAVIGATION HELPERS
// ==========================================

// Haversine formula — calculates straight-line distance between two
// GPS coordinates in meters. Used to check if we've reached a waypoint.
// Named after the haversine trig function it uses internally.
float gpsDistanceMeters(double lat1, double lon1, double lat2, double lon2) {
  const float R = 6371000.0f; // Earth radius in meters
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  float a = sin(dLat/2)*sin(dLat/2) +
            cos(radians(lat1))*cos(radians(lat2))*
            sin(dLon/2)*sin(dLon/2);
  return R * 2.0f * atan2(sqrt(a), sqrt(1.0f-a));
}

// Bearing from point A to point B in degrees (0=north, 90=east, 180=south, 270=west).
// The nav PID uses this to figure out which way to tilt the drone.
float gpsBearing(double lat1, double lon1, double lat2, double lon2) {
  float dLon = radians(lon2 - lon1);
  float y = sin(dLon) * cos(radians(lat2));
  float x = cos(radians(lat1)) * sin(radians(lat2)) -
            sin(radians(lat1)) * cos(radians(lat2)) * cos(dLon);
  float bearing = degrees(atan2(y, x));
  return fmod(bearing + 360.0f, 360.0f); // normalize to 0-360
}

// Decompose a distance+bearing into north/east components.
// "I need to go 10m at bearing 045°" → "7.07m north, 7.07m east"
// This lets us feed two separate PIDs (north and east) instead of
// one combined bearing PID which is harder to tune.
void bearingToNorthEast(float distM, float bearingDeg, float &northM, float &eastM) {
  float bearingRad = radians(bearingDeg);
  northM = distM * cos(bearingRad);
  eastM  = distM * sin(bearingRad);
}

#ifdef WOKWI_SIM
// Reads lines like:
//   HDG:090.0   → simCompassHeading
//   LAT:36.1234 → simGpsLat
//   LON:-80.1234→ simGpsLon
//   FIX:1       → simGpsFix
//   MISSION:1   → triggers the same start-mission logic as the /mission endpoint
// sent from the Wokwi scenario's write-serial steps over the main Serial line.
void parseSimInput() {
  static String buf = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      if (buf.startsWith("HDG:")) {
        withMutex([&]() { simCompassHeading = buf.substring(4).toFloat(); });
      } else if (buf.startsWith("LAT:")) {
        withMutex([&]() { simGpsLat = buf.substring(4).toDouble(); });
      } else if (buf.startsWith("LON:")) {
        withMutex([&]() { simGpsLon = buf.substring(4).toDouble(); });
      } else if (buf.startsWith("FIX:")) {
        withMutex([&]() { simGpsFix = (buf.substring(4).toInt() == 1); });
      } else if (buf.startsWith("MISSION:")) {
        if (buf.substring(8).toInt() == 1) {
          withMutex([&]() {
            navCurrentWP = 0;
            navActive    = true;
            targetAltFt  = WAYPOINTS[0].altFt;
          });
          logLine("[SIM] Mission started via serial injection.");
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
// MUTEX HELPER
//
// Wraps the FreeRTOS mutex grab/release pattern so you don't
// repeat the same 4-line ceremony everywhere.
//
// Usage:
//   withMutex([&]() {
//     gpsLat = gps.location.lat();
//     gpsFix = true;
//   });
// ==========================================
template<typename Fn>
void withMutex(Fn fn) {
  if (xSemaphoreTake(sharedDataMutex, portMAX_DELAY) == pdTRUE) {
    fn();
    xSemaphoreGive(sharedDataMutex);
  }
}

// ==========================================
// LOGGING HELPER
//
// Thread-safe Serial logging. Wraps println with serialMutex
// so log lines never interleave across Core 0/1 boundaries.
// ==========================================
void logLine(const String& msg) {
  if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    Serial.println(msg);
    xSemaphoreGive(serialMutex);
  }
}

// ==========================================
// NAVIGATION HELPERS
// ==========================================

// Zero out all navigation state and targets.
// Call this whenever you want the drone to stop chasing waypoints
// and return to level hover: mission abort, GPS loss, mission complete.
void stopNavigation() {
  withMutex([&]() {
    navActive         = false;
    navTargetRollDeg  = 0.0f;
    navTargetPitchDeg = 0.0f;
  });
}

// ==========================================
// CORE 0: NAVIGATION TASK (~10Hz)
//
// This task runs on Core 0 alongside the web server and WiFi.
// It does three things every loop:
//   1. Reads GPS NMEA sentences from BN-880 over Serial2
//   2. Reads compass heading from QMC5883L over I2C
//   3. If a mission is active, runs the navigation PIDs
//      to compute how much to tilt the drone toward the next waypoint
//
// The output (navTargetRollDeg, navTargetPitchDeg) is picked up
// by the physics task on Core 1 every 5ms.
// ==========================================
void navigationTask(void* parameter) {

  const TickType_t xFrequency = pdMS_TO_TICKS(NAV_LOOP_MS);
  TickType_t lastWakeTime = xTaskGetTickCount();

  float navDt = NAV_LOOP_MS / 1000.0f; // fixed dt for nav PIDs (0.1s at 10Hz)

  for (;;) {

    // ── 1. Feed GPS serial data to TinyGPSPlus ─────────────
    // TinyGPSPlus parses NMEA sentences one character at a time.
    // We drain the serial buffer every loop so we don't fall behind.
    // A full GPS fix sentence arrives once per second at 9600 baud.
#ifdef WOKWI_SIM
    withMutex([&]() {
      gpsLat      = simGpsLat;
      gpsLon      = simGpsLon;
      gpsFix      = simGpsFix;
      gpsSats     = simGpsFix ? 8 : 0;
      gpsSpeedMps = 0.0f;
    });
#else
    while (Serial2.available() > 0) {
      gps.encode(Serial2.read());
    }

    if (gps.location.isValid() && gps.location.age() < 2000) {
      withMutex([&]() {
        gpsLat      = gps.location.lat();
        gpsLon      = gps.location.lng();
        gpsFix      = true;
        gpsSats     = gps.satellites.value();
        gpsSpeedMps = gps.speed.mps();
      });
    } else {
      withMutex([&]() {
        gpsFix = false;
      });
    }
#endif

    // ── 2. Read compass heading ─────────────────────────────
    // QMC5883L returns raw X/Y/Z magnetic field values.
    // The library converts those to a 0-360 degree heading.
    // 0 = magnetic north (close enough to true north for fence line flying).
    //
    // Important: the compass needs calibration for your specific location
    // due to magnetic declination and hard/soft iron distortion from
    // the drone frame. For now we use raw values — add calibration offsets later.
#ifdef WOKWI_SIM
    float heading;
    withMutex([&]() { heading = simCompassHeading; });
#else
    compass.read();
    float heading = compass.getAzimuth();
#endif

    withMutex([&]() {
      compassHeading = heading;
    });

    // ── 3. Navigation PID ───────────────────────────────────
    // Only runs if a mission is active AND we have GPS lock.
    // If we lose GPS mid-flight, navActive goes false and the drone
    // falls back to holding position with attitude PIDs only.
    if (navActive && gpsFix) {

      // Grab current position under mutex
      double currentLat, currentLon;
      int currentWP;

      withMutex([&]() {
        currentLat = gpsLat;
        currentLon = gpsLon;
        currentWP  = navCurrentWP;    
      });

      // Safety check — don't fly if waypoints are exhausted
      if (currentWP >= WAYPOINT_COUNT) {

        // Mission complete — hold position, wait for land command
        stopNavigation();
        logLine("[NAV] Mission complete — leveling out.");

        vTaskDelayUntil(&lastWakeTime, xFrequency);

        continue;
      }

      // Get current target waypoint
      Waypoint wp = WAYPOINTS[currentWP];

      // How far and in what direction is the waypoint?
      float distM   = gpsDistanceMeters(currentLat, currentLon, wp.lat, wp.lon);
      float bearing = gpsBearing(currentLat, currentLon, wp.lat, wp.lon);

      // Have we arrived? Advance to next waypoint.
      if (distM < WAYPOINT_ACCEPT_RADIUS_M) {
        String wpReachedLog = String("[NAV] Reached waypoint ") + String(currentWP) +
                              " — advancing to waypoint " + String(currentWP + 1);
        logLine(wpReachedLog);

        withMutex([&]() {
            navCurrentWP++;
            targetAltFt = (navCurrentWP < WAYPOINT_COUNT)
                          ? WAYPOINTS[navCurrentWP].altFt
                          : wp.altFt; // hold last altitude when mission complete
        });

        vTaskDelayUntil(&lastWakeTime, xFrequency);

        continue;
      }

      // Decompose distance into north/east components
      // North = forward/backward tilt (pitch axis)
      // East  = left/right tilt (roll axis)
      float northM, eastM;
      bearingToNorthEast(distM, bearing, northM, eastM);

      // Run navigation PIDs
      // Error is how far we need to go in each direction.
      // Output is how many degrees to tilt in that direction.
      // The physics task's roll/pitch PIDs then chase that tilt angle.
      float targetPitch = navNorthPID.computeWithError(northM, navDt); // nose forward = positive
      float targetRoll  = navEastPID.computeWithError(eastM,  navDt);  // right = positive

      // Write navigation outputs to shared state for physics task
        withMutex([&]() {
          navTargetRollDeg  = targetRoll;
          navTargetPitchDeg = targetPitch;
          navDistToWP       = distM;
          navBearingToWP    = bearing;
        });

      String navWPLog = String("[NAV] WP=") + String(currentWP) +
                        " dist=" + String(distM, 1) +
                        "m bearing=" + String(bearing, 1) +
                        "° → pitchTarget=" + String(targetPitch, 2) +
                        "° rollTarget=" + String(targetRoll, 2);
      logLine(navWPLog);

    } else {
      // No mission active — zero out navigation targets
      // Physics task will use 0.0f (hover level) for roll/pitch
      withMutex([&]() {
        navTargetRollDeg  = 0.0f;
        navTargetPitchDeg = 0.0f;
      });
    }

    vTaskDelayUntil(&lastWakeTime, xFrequency);
  }
}

// ==========================================
// CORE 1: PHYSICS + FLIGHT TASK (~200Hz)
// ==========================================
unsigned long lastGyroMicros = 0;

void physicsTask(void* parameter) {

  // get the current time in microseconds for delta time calculations
  lastGyroMicros = micros();

  // At 10000ms (0.1Hz):
  // - The filter expects one tap every 10 seconds. So each time you call updateIMU, it thinks "okay, 10 seconds passed, let me integrate the gyroscope over 10 seconds to get the new angle." Slow and sluggish but correct for that rate.
  // At 5ms (200Hz):
  // - The filter expects 200 taps per second. So each updateIMU it thinks "okay, 5 milliseconds passed, tiny integration step." Very responsive and smooth.
  // The key thing is — the filter doesn't actually know what time it is. It's blind. All it knows is "each tap = 1/HZ seconds passed." So if you lie to it by saying 200Hz but only actually call it once every 10 seconds, it thinks tiny sips of time are passing when actually huge gulps are. Your angles will barely move even if you spin the drone around.
  // That's why keeping PHYSICS_LOOP_MS and PHYSICS_LOOP_HZ in sync matters — you're just making sure the filter's internal clock matches reality.
  filter.begin(PHYSICS_LOOP_HZ);
  const TickType_t xFrequency = pdMS_TO_TICKS(PHYSICS_LOOP_MS);

  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {

    // at the end of each loop, we have a determinsitic delay to maintain a fixed loop frequency
    // but, we also want to measure the actual time between loops for physics calculations, so we use micros() to get the actual delta time
    unsigned long now = micros();

    // measure actual delta that passed since last loop, in seconds
    float dt = (now - lastGyroMicros) / 1000000.0f;
    lastGyroMicros = now;

    // so an MPU can get the current acceleration, spin rate, and temperature
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // convert gyro readings from rad/s → deg/s for Madgwick filter
    float gx = g.gyro.x * 57.2958f;
    float gy = g.gyro.y * 57.2958f;
    float gz = g.gyro.z * 57.2958f;

    // If the delta time is reasonable, update the Madgwick filter with the new gyro and accelerometer readings.
    // If the delta time is too large (e.g., due to a long delay), we skip the update to avoid instability in the filter.
    // In practice the jitter is tiny. Your FreeRTOS task is running on a deterministic scheduler — vTaskDelayUntil is specifically designed to keep the loop as close to exactly 10ms as possible. You might get 10.001ms one loop, 9.998ms the next. That's so small that the error in angle = spin_rate × time is basically invisible.
    // The only time it becomes a real problem is if the loop freezes for a long time — like half a second or more. That's what the dt < 1.0f guard is catching. Not jitter, just catastrophic stalls.
    // So the mental model is:
    // Tiny timing variations → Madgwick handles it fine, don't worry
    // Complete stall → skip that reading entirely with the guard
    // If you were running on bare metal with no scheduler and loops that could swing wildly between 5ms and 50ms, then yeah you'd need to feed the real dt in somehow. But with FreeRTOS keeping you pinned to a fixed rate, just telling the filter "expect 100Hz" is accurate enough.
    if (dt > 0 && dt < 1.0f)
    {
      filter.updateIMU(gx, gy, gz, a.acceleration.x, a.acceleration.y, a.acceleration.z);
    }

    float currentRoll  = filter.getRoll();
    float currentPitch = filter.getPitch();
    float currentYaw   = filter.getYaw();

    // ── 3. Read altitude (meters → feet) ───────────────────
    float altitudeFt = ((float)barometer.readAltitudeMeters() * 3.28084f) - groundAltitudeFt;

    // ── 4. Get navigation targets from shared state ─────────
    // If a waypoint mission is running, the nav task on Core 0
    // has computed how much to tilt the drone toward the next waypoint.
    // We use those target angles instead of 0.0f (level hover).
    // If no mission is active, these are 0.0f and the drone just hovers level.
    float targetRollDeg, targetPitchDeg, currentCompassHeading;
    bool  missionActive;

    withMutex([&]() {
      targetRollDeg         = navTargetRollDeg;
      targetPitchDeg        = navTargetPitchDeg;
      missionActive         = navActive;
      currentCompassHeading = compassHeading;
    });

    // ── 5. Flight control ───────────────────────────────────
    float m1 = 0, m2 = 0, m3 = 0, m4 = 0;
    float baseThrottle    = 0;
    float rollCorrection  = 0;
    float pitchCorrection = 0;
    float yawCorrection   = 0;

    if (flightEnabled) {

      // Altitude PID → base throttle for all motors
      // "How far from target? Spin all motors faster/slower."
      baseThrottle = altitudePID.compute(targetAltFt, altitudeFt, dt);

      // Roll PID → left/right motor trim
      // When hovering: target = 0° (level)
      // When navigating: target = navTargetRollDeg (tilt toward waypoint)
      rollCorrection = rollPID.compute(targetRollDeg, currentRoll, dt);

      // Pitch PID → front/rear motor trim
      // When hovering: target = 0° (level)
      // When navigating: target = navTargetPitchDeg (tilt toward waypoint)
      pitchCorrection = pitchPID.compute(targetPitchDeg, currentPitch, dt);

      // Yaw PID → diagonal motor trim to rotate drone back to target heading
      //
      // Previously this used filter.getYaw() which drifts over time because
      // the gyroscope accumulates small errors. Now we use the compass heading
      // from the BN-880 which always knows true magnetic north — no drift ever.
      //
      // The compass needle analogy:
      //   Think of a slim piece of paper standing vertical on a table,
      //   spine pointing straight up. The Z-axis compass lies flat at the top.
      //   When the drone yaws (spins left or right), that flat compass needle
      //   rotates. The gyroscope measures how fast the needle is moving.
      //   The compass tells us where it actually points.
      //
      // P — how far from north right now?
      //     "90 degrees off → spin hard"
      // I — _integral accumulates (error × dt) every loop
      //     grows larger the longer we stay off heading
      //     "been drifting west for a while, probably wind → spin a bit harder"
      // D — how fast is the error shrinking?
      //     "already rotating back to north fast → ease off before we overshoot east"
      //
      // output: single float fed into motor mixer to spin the drone left or right
      float targetHeading = 0.0f; // true north — replace with desired fence line heading
      float yawError = targetHeading - currentCompassHeading;
      // Wraparound fix — compass goes 0-360, so crossing north (359→1) would
      // give a huge error without this. We always take the shortest path.
      if (yawError > 180.0f)  yawError -= 360.0f;
      if (yawError < -180.0f) yawError += 360.0f;
      yawCorrection = yawPID.computeWithError(yawError, dt);

      // Motor mixer: combine base throttle + stabilization corrections
      //
      //      FRONT
      //  M1 (FL)   M2 (FR)
      //  M3 (RL)   M4 (RR)
      //      REAR
      //
      // Roll +  → tilt right → left motors need more power
      // Pitch + → nose up    → front motors need more power
      // Yaw +   → spin CW   → diagonal pair (FL+RR) spin faster, (FR+RL) slower
      m1 = baseThrottle + pitchCorrection + rollCorrection - yawCorrection;  // FL
      m2 = baseThrottle + pitchCorrection - rollCorrection + yawCorrection;  // FR
      m3 = baseThrottle - pitchCorrection + rollCorrection + yawCorrection;  // RL
      m4 = baseThrottle - pitchCorrection - rollCorrection - yawCorrection;  // RR

      // Clamp all motors to valid range
      m1 = constrain(m1, 0.0f, 1.0f);
      m2 = constrain(m2, 0.0f, 1.0f);
      m3 = constrain(m3, 0.0f, 1.0f);
      m4 = constrain(m4, 0.0f, 1.0f);

      // esc1.write(m1);
      // esc2.write(m2);
      // esc3.write(m3);
      // esc4.write(m4);
    } else {
      // Flight disabled — all motors off
      altitudePID.reset();
      rollPID.reset();
      pitchPID.reset();
      navNorthPID.reset();
      navEastPID.reset();
      // esc1.disarm(); esc2.disarm(); esc3.disarm(); esc4.disarm();
    }

    String physicsLog = String("[PHYSICS] pitch=") + String(currentPitch, 2) +
                        " deg, altFt=" + String(altitudeFt, 2) +
                        " ft, dt=" + String(dt, 4) +
                        " s, rollCorr=" + String(rollCorrection, 3) +
                        " pitchCorr=" + String(pitchCorrection, 3) +
                        " m1=" + String(m1, 2) +
                        " m2=" + String(m2, 2) +
                        " m3=" + String(m3, 2) +
                        " m4=" + String(m4, 2);
    logLine(physicsLog);

    // ── 6. Write to shared state ────────────────────────────
    withMutex([&]() {
      sharedRoll  = currentRoll;
      sharedPitch = currentPitch;
      sharedYaw   = currentYaw;
      sharedAccX  = a.acceleration.x;
      sharedAccY  = a.acceleration.y;
      sharedAccZ  = a.acceleration.z;
      sharedTemp  = temp.temperature;
      sharedAltitudeFt      = altitudeFt;
      sharedM1 = m1; sharedM2 = m2; sharedM3 = m3; sharedM4 = m4;
      sharedBaseThrottle    = baseThrottle;
      sharedRollCorrection  = rollCorrection;
      sharedPitchCorrection = pitchCorrection;
    }); 

    vTaskDelayUntil(&lastWakeTime, xFrequency);
  }
}

// ==========================================
// WEB FORMATTERS
// ==========================================
String getGyroReadings() {
  float r, p, y;
  withMutex([&]() {
    r = sharedRoll; p = sharedPitch; y = sharedYaw;
  });
  readings["gyroX"] = r;
  readings["gyroY"] = p;
  readings["gyroZ"] = y;
  String out; serializeJson(readings, out); return out;
}

String getAccReadings() {
  float ax, ay, az;
  withMutex([&]() {
    ax = sharedAccX; ay = sharedAccY; az = sharedAccZ;
  });
  readings["accX"] = ax; readings["accY"] = ay; readings["accZ"] = az;
  String out; serializeJson(readings, out); return out;
}

String getFlightReadings() {
  float alt, m1, m2, m3, m4, base, roll, pitch;
  bool  fix;
  double lat, lon;
  float  heading, dist, bearing;
  int    wp, sats;

  withMutex([&]() {
    alt     = sharedAltitudeFt;
    m1      = sharedM1; m2 = sharedM2; m3 = sharedM3; m4 = sharedM4;
    base    = sharedBaseThrottle;
    roll    = sharedRollCorrection;
    pitch   = sharedPitchCorrection;
    fix     = gpsFix;
    lat     = gpsLat;
    lon     = gpsLon;
    sats    = gpsSats;
    heading = compassHeading;
    dist    = navDistToWP;
    bearing = navBearingToWP;
    wp      = navCurrentWP;
  });

  readings["altFt"]          = alt;
  readings["targetFt"]       = targetAltFt;
  readings["m1"]             = (int)(m1 * 100);
  readings["m2"]             = (int)(m2 * 100);
  readings["m3"]             = (int)(m3 * 100);
  readings["m4"]             = (int)(m4 * 100);
  readings["baseThrottle"]   = (int)(base  * 100);
  readings["rollCorrection"]  = (int)(roll  * 100);
  readings["pitchCorrection"] = (int)(pitch * 100);
  readings["flightEnabled"]  = flightEnabled;
  readings["gpsFix"]         = fix;
  readings["gpsLat"]         = lat;
  readings["gpsLon"]         = lon;
  readings["gpsSats"]        = sats;
  readings["compassHeading"] = heading;
  readings["navActive"]      = navActive;
  readings["navWaypoint"]    = wp;
  readings["navDistM"]       = dist;
  readings["navBearing"]     = bearing;

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

void initCompass() {
  // QMC5883L compass inside the BN-880
  // Shares the I2C bus already started with Wire.begin(16, 15)
  logLine("[COMPASS] Initializing QMC5883L...");
  compass.init();
  compass.setMode(0x01, 0x0C, 0x10, 0xC0); // continuous, 200Hz, 8G, 512 OSR
  logLine("[COMPASS] QMC5883L ready.");
}

void initGPS() {
  // BN-880 GPS talks over Serial2 (UART2 on ESP32-S3)
  // NMEA sentences at 9600 baud by default
  logLine("[GPS] Initializing BN-880 GPS on Serial2...");
  Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  logLine("[GPS] Waiting for satellite fix \u2014 this can take 30-90 seconds outdoors.");
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
  Wire.begin(16, 15); // SDA=16, SCL=15 — shared I2C bus for MPU6050 + BME280 + QMC5883L

  logLine("=== BOOT START ===");

  initWiFi();
  initLittleFS();
  initMPU();
  #ifndef WOKWI_SIM
    initCompass();
    initGPS();
  #endif
  barometer.initialize();
  // esc1.init(4, RMT_CHANNEL_0); // M1 front-left
  // esc2.init(5, RMT_CHANNEL_1); // M2 front-right
  // esc3.init(6, RMT_CHANNEL_2); // M3 rear-left
  // esc4.init(7, RMT_CHANNEL_3); // M4 rear-right
  logLine("[ESC] DShot600 ready on GPIO 4/5/6/7");

  groundAltitudeFt = (float)barometer.readAltitudeMeters() * 3.28084f;

  // Navigation task on Core 0 — alongside WiFi and web server
  // Priority 1 (lower than physics) — GPS parsing can slip a few ms without consequence
  xTaskCreatePinnedToCore(navigationTask, "NavTask",     8192, NULL, 1, NULL, 0);

  // Physics task on Core 1 — isolated real-time math, nothing else on this core
  // Priority 2 (higher than nav) — motors must never miss a beat
  xTaskCreatePinnedToCore(physicsTask,    "PhysicsTask", 8192, NULL, 2, NULL, 1);

  // ── Routes ──────────────────────────────────────────────

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* r) {
    r->send(LittleFS, "/index.html", "text/html");
  });

  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  // Enable autonomous flight with target altitude in feet
  // e.g. /fly?ft=10
  server.on("/fly", HTTP_GET, [](AsyncWebServerRequest* r) {
    if (r->hasParam("ft")) {
      float ft = r->getParam("ft")->value().toFloat();
      if (ft < 0)    ft = 0;
      if (ft > 50)   ft = 50; // safety cap at 50ft
      targetAltFt   = ft;
      flightEnabled = true;
      logLine(String("[FLIGHT] Target: ") + String(ft) + " ft");
    }
    r->send(200, "text/plain", "OK");
  });

  // Start autonomous fence line mission
  // Drone must already be flying (/fly first), then /mission starts waypoint following
  // e.g. /mission
  server.on("/mission", HTTP_GET, [](AsyncWebServerRequest* r) {
    if (!gpsFix) {
      r->send(400, "text/plain", "NO GPS FIX — wait for satellite lock before starting mission");
      return;
    }
    navCurrentWP = 0;
    navActive    = true;
    targetAltFt  = WAYPOINTS[0].altFt;
    logLine("[MISSION] Fence line mission started.");
    r->send(200, "text/plain", "MISSION STARTED");
  });

  // Abort mission — drone holds position at current altitude
  server.on("/abort", HTTP_GET, [](AsyncWebServerRequest* r) {
    stopNavigation();
    logLine("[MISSION] Mission aborted — holding position.");
    r->send(200, "text/plain", "MISSION ABORTED");
  });

  // Land: target drops to 0, flight stays enabled so PID descends gently
  server.on("/land", HTTP_GET, [](AsyncWebServerRequest* r) {
    stopNavigation();
    targetAltFt   = 0.0f;  // ← this went missing
    flightEnabled = true;
    r->send(200, "text/plain", "LANDING");
  });

  // Emergency stop — cuts motors immediately
  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest* r) {
    stopNavigation();       // zeros navActive + roll/pitch targets
    flightEnabled = false;
    targetAltFt   = 0.0f;
    r->send(200, "text/plain", "STOPPED");
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

  // Orientation → cube rotation
  if (millis() - lastGyroSend > SSE_GYRO_MS) {
    events.send(getGyroReadings().c_str(), "gyro_readings", millis());
    lastGyroSend = millis();
  }
  // Accelerometer
  if (millis() - lastAccSend > SSE_ACC_MS) {
    events.send(getAccReadings().c_str(), "accelerometer_readings", millis());
    lastAccSend = millis();
  }
  // Flight data: altitude, motors, PID debug, GPS, nav status
  if (millis() - lastFlightSend > SSE_FLIGHT_MS) {
    events.send(getFlightReadings().c_str(), "flight_readings", millis());
    lastFlightSend = millis();
  }
}
