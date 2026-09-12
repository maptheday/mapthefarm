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
#include "FlightConfig.hpp"
#include "FlightModel.hpp"
#include "SensorTypes.hpp"
#include "ControlTypes.hpp"
#include "PhaseState.hpp"
#include "HilState.hpp"
#include "MotorController.hpp"
#include "PID.hpp"
#include "IFlightPhase.hpp"
#include "FlightRuntime.hpp"
#include "PhaseRegistry.hpp"

const char* ssid     = "SpectrumSetup-C3";
const char* password = "mellowlemon735";

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
// ESC (DShot600, via EspESC.hpp)
// ==========================================
// EspESC.hpp is DShot600 (digital, no calibration step, no PWM pulse
// width concept). ESC_PULSE_MAX_US/MIN_US/ARM_US below were written for
// analog/OneShot PWM ESCs and don't apply -- kept only so nothing else
// in this file that might still reference them breaks; DShot's real
// throttle range (48-2047) is handled internally by EspESC::write().
// "Disarmed" is motors[i].disarm() (DShot command 0), not a min-throttle
// float constant -- there's no PWM-style minimum to hold in DShot.
const int   ESC_PULSE_MAX_US = 2000;  // unused -- DShot has no PWM pulse width
const int   ESC_PULSE_MIN_US = 1000;  // unused -- DShot has no PWM pulse width
const int   ESC_PULSE_ARM_US = 1000;  // unused -- DShot has no arm-pulse step

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
// PLACEHOLDER pin -- moved off GPIO16 because it collided with
// Wire.begin(16, 15) (I2C SDA). GPIO4 is clear of every pin this file
// knows about (I2C: 16/15, GPS: 17/18), but EspESC.hpp's pin usage is
// NOT visible from this file -- confirm GPIO4 is actually free on your
// board before wiring the receiver. See the boot-time warning below.
// GPIO8 is confirmed clear of every pin this project defines: I2C
// (16/15), GPS (17/18), and ESC M1-M4 (4/5/6/7, per EspESC.hpp). Still
// worth a final visual check against your actual board silkscreen --
// GPIO8 is unused on most ESP32-S3 DevKitC-1 boards but isn't a
// hardware-enforced guarantee the way the others above are.
// ==========================================
// HARDWARE
// ==========================================
Adafruit_MPU6050 mpu;
Madgwick         filter;
EspBarometer     barometer;
TinyGPSPlus      gps;
QMC5883LCompass  compass;

#ifndef WOKWI_SIM
AsyncWebServer    server(80);
// SSE endpoint for live telemetry -- was never declared, so getGyroReadings()/
// getAccReadings()/getFlightReadings() had no way to reach the frontend.
// NOTE: event names below ("gyro","acc","flight") are a best guess from the
// formatter function names -- verify against your frontend's EventSource
// listener names (data/*.js) before relying on this.
AsyncEventSource  events("/events");
#endif

// EspESC.hpp is instance-based (one object per motor), not static calls --
// the previous EspESC::begin()/writeAllMicroseconds()/writeMotor() calls
// throughout this file don't exist on that class and wouldn't have compiled.
// Pins and M1-M4 ordering are exactly what EspESC.hpp's own header comment
// documents; RMT_CHANNEL_0-3 are the 4 channels this legacy driver/rmt.h
// API exposes on ESP32-S3. writeMotorMix()/disarmAllMotors() are defined
// further down, right after the MotorMix struct they depend on.
EspESC motors[4]; // index 0=M1, 1=M2, 2=M3, 3=M4
const int         MOTOR_PINS[4]     = { 4, 5, 6, 7 };
const rmt_channel_t MOTOR_RMT_CH[4] = { RMT_CHANNEL_0, RMT_CHANNEL_1, RMT_CHANNEL_2, RMT_CHANNEL_3 };

MotorController motorController;

// ==========================================
// PANIC (macro now in FlightRuntime.hpp)
// ==========================================
#define ASSERT_MOTORS_INITIALIZED(d) \
  do { if ((d).m1 == 0.0f && (d).m2 == 0.0f && (d).m3 == 0.0f && (d).m4 == 0.0f) \
    PANIC("control decision on uninitialized motor outputs"); } while(0)

// transitionTo() is declared in FlightRuntime.hpp (with its default argument).

#ifdef WOKWI_SIM
bool hilGateAllows(FlightPhase next, TransitionReason reason) {
  if (!hilGatePending) {
    hilGatePending = true;
    hilGateApproved = false;
    hilGateNext = next;
    hilGateReason = reason;
    logLine(String("[HIL_GATE] request=") + phaseName(next) +
            " reason=" + reasonName(reason));
    return false;
  }
  return hilGateNext == next && hilGateApproved;
}

bool hilGateBlocked() {
  if (!hilGatePending) return false;
  if (hilGateApproved) {
    FlightPhase next = hilGateNext;
    TransitionReason reason = hilGateReason;
    transitionTo(next, reason);
    hilGatePending = false;
    hilGateApproved = false;
    hilGateReason = REASON_NONE;
  }
  return true;
}
#endif

// ==========================================
// SHARED STATE (repository)
// ------------------------------------------
// Per-phase state blocks, SharedState, and withMutex() live in PhaseState.hpp.
// The single instances are DEFINED here (the .ino is the one place that owns
// them); PhaseState.hpp / FlightRuntime.hpp declare them `extern` for phases.
// ==========================================
SemaphoreHandle_t sharedDataMutex;
SemaphoreHandle_t serialMutex;

volatile SharedState shared;
float groundAltitudeFt = 0;

void logLine(const String& msg) {
  if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    Serial.println(msg);
    Serial.flush(); // block until this line is actually on the wire before
                     // releasing the mutex -- USB CDC buffers writes
                     // asynchronously, so without this a second task can
                     // start writing while this line is still draining,
                     // tearing the two messages together.
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

// Sends a computed MotorMix straight to all 4 ESCs. Every flying phase's
// physicsTick() calls this right after MotorController computes the mix.
void writeMotorMix(const MotorMix& mix) {
#ifndef WOKWI_SIM
  motors[0].write(mix.m1);
  motors[1].write(mix.m2);
  motors[2].write(mix.m3);
  motors[3].write(mix.m4);
#else
  (void)mix;
#endif
}
// Cuts all 4 motors immediately. Used by the Parked/Landed phases instead
// of a PWM "min throttle" concept that doesn't exist in DShot.
void disarmAllMotors() {
#ifndef WOKWI_SIM
  for (int i = 0; i < 4; i++) motors[i].disarm();
#endif
}

// ==========================================
// FUNCTIONAL STATE SETTERS / TRANSITIONS
// ==========================================
void transitionTo(FlightPhase next, TransitionReason reason) {
#ifdef WOKWI_SIM
  if (!hilGateAllows(next, reason)) return;
#endif
  withMutex([&]() {
    // Build the entry context ONCE here (the orchestrator's job), including
    // carrying the arm time + launch point forward from the previous flight
    // phase. Each phase's onEnter() then only sets up its OWN state.
    EnterContext ctx;
    ctx.prevPhase         = shared.phase;
    ctx.now               = millis();
    ctx.currentAltFt      = shared.raw.baroAltitudeFt;
    ctx.currentHeadingDeg = shared.raw.compassHeadingDeg;
    ctx.currentLat        = shared.raw.gps.lat;
    ctx.currentLon        = shared.raw.gps.lon;
  #ifdef WOKWI_SIM
    ctx.currentLat        = simGpsLat;
    ctx.currentLon        = simGpsLon;
  #endif

    switch (ctx.prevPhase) {
      case PHASE_RAISE:
        ctx.carriedArmedAtMs = shared.trip_raise.armedAtMs;
        ctx.carriedLaunchLat = shared.trip_raise.launchLat;
        ctx.carriedLaunchLon = shared.trip_raise.launchLon;
        break;
      case PHASE_HOLD:
        ctx.carriedArmedAtMs = shared.trip_hold.armedAtMs;
        ctx.carriedLaunchLat = shared.trip_hold.launchLat;
        ctx.carriedLaunchLon = shared.trip_hold.launchLon;
        break;
      case PHASE_MISSION:
        ctx.carriedArmedAtMs = shared.trip_mission.armedAtMs;
        ctx.carriedLaunchLat = shared.trip_mission.launchLat;
        ctx.carriedLaunchLon = shared.trip_mission.launchLon;
        break;
      case PHASE_RTL:
        ctx.carriedArmedAtMs = shared.trip_rtl.armedAtMs;
        ctx.carriedLaunchLat = shared.trip_rtl.launchLat;
        ctx.carriedLaunchLon = shared.trip_rtl.launchLon;
        break;
      default: break;
    }

    // Hand off to the phase we're entering. onEnter runs under this lock.
    phaseFor(next)->onEnter(ctx);

    shared.phase = next;
    shared.transitionReason = reason;
  });
}

// Safety wrapper to avoid repetitive checks
void checkCoreFailsafes(unsigned long armedAtMs, double launchLat, double launchLon) {
  bool tripRTL = false;
  TransitionReason rtlReason = REASON_NONE;
  
  // 1. Max Flight Time
  if (armedAtMs > 0 && (millis() - armedAtMs) >= MAX_FLIGHT_TIME_MS) {
    logLine("[SAFETY] Max flight time reached — forcing RTL.");
    tripRTL = true;
    rtlReason = REASON_MAX_FLIGHT_TIME;
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
#ifdef WOKWI_SIM
  fix      = simGpsFix;
  lat      = simGpsLat;
  lon      = simGpsLon;
  lastFix  = fix ? millis() : lastFix;
#endif
  
#ifdef WOKWI_SIM
  if (launchLat != 0.0) {
#else
  if (fix && launchLat != 0.0) {
#endif
    if (gpsDistanceMeters(lat, lon, launchLat, launchLon) > GEOFENCE_RADIUS_M) {
      logLine("[SAFETY] Geofence exceeded — forcing RTL.");
      tripRTL = true;
      if (rtlReason == REASON_NONE) rtlReason = REASON_GEOFENCE;
    }
  }

  // 3. GPS Loss
  if (!fix && lastFix > 0 && (millis() - lastFix) >= GPS_LOSS_ABORT_MS) {
      logLine("[SAFETY] GPS fix lost — aborting directly to LANDING.");
      transitionTo(PHASE_LANDING, REASON_GPS_LOSS); // Can't RTL without GPS
      return;
  }

  FlightPhase phase;
  withMutex([&]() { phase = shared.phase; });

  if (tripRTL && phase != PHASE_RTL && phase != PHASE_LANDING && phase != PHASE_LANDED) {
    transitionTo(PHASE_RTL, rtlReason);
  }
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
    if (hilGateBlocked()) {
      vTaskDelay(pdMS_TO_TICKS(NAV_LOOP_MS));
      continue;
    }
#endif
#ifdef WOKWI_SIM
    withMutex([&]() {
  shared.raw.gps.lat = simGpsLat;
  shared.raw.gps.lon = simGpsLon;
  shared.raw.gps.fix = simGpsFix;
  if (simGpsFix) shared.raw.gps.lastFixMs = millis();
  shared.raw.gps.sats = simGpsFix ? 8 : 0;
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

    phaseFor(phase)->navTick(navDt);
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
#ifdef WOKWI_SIM
    if (hilGateBlocked()) {
      vTaskDelay(pdMS_TO_TICKS(PHYSICS_LOOP_MS));
      continue;
    }
#endif
    unsigned long now = micros();
    float dt          = (now - lastGyroMicros) / 1000000.0f;
    lastGyroMicros    = now;

#ifndef WOKWI_SIM
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
      shared.raw.imu.gyroX      = filter.getRoll();
      shared.raw.imu.gyroY      = filter.getPitch();
      shared.raw.imu.gyroZ      = filter.getYaw();
      shared.raw.imu.accX       = a.acceleration.x;
      shared.raw.imu.accY       = a.acceleration.y;
      shared.raw.imu.accZ       = a.acceleration.z;
      shared.raw.imu.temp       = temp.temperature;
      shared.raw.baroAltitudeFt = baroAlt;
    });
#else
    // Sim mode: no MPU6050 or barometer hardware present.
    // IMU is left at zero -- attitude control is not exercised in HIL tests.
    // baroAltitudeFt is driven by the HIL runner via ALT: serial commands
    // and written directly into shared.raw by parseSimInput().
    (void)dt;
#endif

    FlightPhase phase;
    withMutex([&]() { phase = shared.phase; });

    phaseFor(phase)->physicsTick(dt);
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

  // Each phase fills in its own telemetry fields.
  phaseFor(phase)->writeTelemetry(readings);

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
    compassScaleX = compassPrefs.getFloat("sclX", 1);
    compassScaleY = compassPrefs.getFloat("sclY", 1);
    compassScaleZ = compassPrefs.getFloat("sclZ", 1);
  }
  compassPrefs.end();
  compass.setCalibration(compassOffsetX, compassOffsetY, compassOffsetZ,
                         compassScaleX, compassScaleY, compassScaleZ);
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

// DShot600 init runs every boot: bring up each motor's RMT channel,
// then send an explicit disarm. No calibration/beep sequence -- that
// was a leftover PWM-ESC description; DShot has no equivalent step.
void initESC() {
  // DShot600 has NO calibration step -- the previous "send max, then
  // min, then arm" sequence is an analog/OneShot PWM ESC ritual and
  // does nothing meaningful (or could be actively wrong) on a DShot ESC.
  // All that's needed is bringing up each motor's RMT channel and
  // sending an explicit disarm so they're confirmed at zero before
  // anything else runs.
  for (int i = 0; i < 4; i++) {
    motors[i].init(MOTOR_PINS[i], MOTOR_RMT_CH[i]);
  }
  disarmAllMotors();
  logLine("[ESC] DShot600 channels initialized, all motors disarmed.");
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
// Defined further down (shared with the real crsfTask, which is compiled
// out under WOKWI_SIM). Forward-declared here so parseSimInput can call
// them without reordering the whole file.
void crsfHandleStart();
void crsfHandleStop();

void resetSimState() {
  simGpsFix = false;
  hilGatePending = false;
  hilGateApproved = false;
  hilGateNext = PHASE_PARKED;
  hilGateReason = REASON_NONE;
  withMutex([&]() {
    shared.phase = PHASE_PARKED;
    shared.transitionReason = REASON_NONE;
    shared.raw.gps.fix = false;
    shared.raw.gps.lastFixMs = 0;
    shared.trip_mission.currentWP = 0;
    shared.trip_mission.active = false;
  });
  logLine("[HIL] State reset.");
}

void parseSimInput() {
  static String buf = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      if (buf.startsWith("HDG:")) { 
        withMutex([&]() { shared.raw.compassHeadingDeg = buf.substring(4).toFloat(); }); 
      }
      else if (buf.startsWith("LAT:")) {
        simGpsLat = buf.substring(4).toDouble();
      }
      else if (buf.startsWith("LON:")) {
        simGpsLon = buf.substring(4).toDouble();
      }
      else if (buf.startsWith("FIX:")) {
        simGpsFix = buf.substring(4).toInt() == 1;
      }
      else if (buf.startsWith("ALT:")) {
        withMutex([&]() { shared.raw.baroAltitudeFt = buf.substring(4).toFloat(); });
      }
      else if (buf.startsWith("MISSION:")) {
        bool fixNow; 
        withMutex([&]() { fixNow = shared.raw.gps.fix; });
        if (fixNow) transitionTo(PHASE_MISSION);
      }
      // Fake RC switch commands -- call the exact same handlers the real
      // crsfTask calls on a real CRSF frame. Each CRSFSTART:1 line is one
      // simulated LOW->HIGH edge (send it once per "press", not held).
      else if (buf.startsWith("CRSFSTART:")) {
        if (buf.substring(10).toInt() == 1) crsfHandleStart();
      }
      else if (buf.startsWith("CRSFSTOP:")) {
        if (buf.substring(9).toInt() == 1) crsfHandleStop();
      }
      // HIL runner ping -- confirms firmware is alive and ready.
      // Runner sends PING: after reconnecting; firmware echoes back
      // [HIL] Ready so the runner knows setup() has completed.
      else if (buf.startsWith("PING:")) {
        logLine("[HIL] Ready.");
      }
      else if (buf.startsWith("RESET:")) {
        resetSimState();
      }
      else if (buf.startsWith("ALLOW:")) {
        String allowed = buf.substring(6);
        for (int i = PHASE_PARKED; i <= PHASE_LANDED; ++i) {
          FlightPhase phase = static_cast<FlightPhase>(i);
          if (allowed == phaseName(phase) && hilGatePending && hilGateNext == phase) {
            hilGateApproved = true;
            logLine(String("[HIL_GATE] approved=") + allowed);
            break;
          }
        }
      }
      // On-demand motor telemetry -- queried by the HIL harness instead of
      // pushed periodically, so it never collides on the wire with
      // safety/phase-transition log lines fired from the nav task.
      else if (buf.startsWith("MOTOR?")) {
        Dashboard_RTL m;
        withMutex([&]() { m = shared.dashboard_rtl; });
        logLine("[MOTOR] base=" + String(m.baseThrottle, 2) +
                " roll=" + String(m.rollCorrection, 3) +
                " pitch=" + String(m.pitchCorrection, 3));
      }
      // HIL state query. Tests must not use human-readable event logs as a
      // control protocol because a dropped CDC byte can make a real event
      // indistinguishable from a missing event.
      else if (buf.startsWith("STATUS?")) {
        String requestId = buf.substring(7);
        if (requestId.length() == 0) requestId = "0";
        FlightPhase phase;
        RTLState rtlState;
        bool gatePending;
        FlightPhase gateNext;
        TransitionReason transitionReason;
        int waypoint;
        withMutex([&]() {
          phase = shared.phase;
          rtlState = shared.trip_rtl.state;
          gatePending = hilGatePending;
          gateNext = hilGateNext;
          transitionReason = shared.transitionReason;
          waypoint = shared.trip_mission.currentWP;
        });
        TransitionReason reason = gatePending ? hilGateReason : transitionReason;
        logLine("[STATUS] id=" + requestId +
                " phase=" + String(phaseName(phase)) +
                " rtl=" + String(rtlState == RTL_CLIMB ? "CLIMB" :
                                    rtlState == RTL_RETURN ? "RETURN" : "SETTLE") +
                " gate=" + String(gatePending ? phaseName(gateNext) : "NONE") +
                " reason=" + reasonName(reason) +
                " wp=" + String(waypoint));
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

// Shared by the real crsfTask (byte-parsed CRSF frames) and, under
// WOKWI_SIM, by parseSimInput() (fake "CRSFSTOP:1" serial commands).
// Keeping this in one place means the sim exercises the exact same
// phase-transition logic as real hardware -- only the byte-level
// frame parsing itself goes untested in sim.
void crsfHandleStop() {
  FlightPhase phase;
  withMutex([&]() { phase = shared.phase; });
  if (phase != PHASE_PARKED && phase != PHASE_LANDED) {
    logLine("[CRSF] STOP switch -- emergency stop.");
    transitionTo(PHASE_PARKED, REASON_EMERGENCY_STOP);
  }
}

void crsfHandleStart() {
  FlightPhase phase;
  bool hasFix;
  withMutex([&]() { phase = shared.phase; hasFix = shared.raw.gps.fix; });

  // LANDED is treated the same as PARKED for re-arming: motors are
  // already confirmed at min throttle in both (see the Parked/Landed phases'
  // physicsTick), so there's no safety reason to force a trip
  // back through the web /stop endpoint just to fly again after a
  // normal landing. (Previously LANDED fell through to "already
  // flying/busy", which was wrong -- a landed drone isn't busy.)
  if (phase == PHASE_PARKED || phase == PHASE_LANDED) {
    if (!hasFix) {
      logLine("[CRSF] START ignored -- no GPS fix.");
    } else {
      logLine("[CRSF] START switch -- arming and taking off.");
      transitionTo(PHASE_RAISE, REASON_OPERATOR_START);
    }
  } else if (phase == PHASE_HOLD) {
     logLine("[CRSF] START switch -- starting waypoint mission.");
     transitionTo(PHASE_MISSION, REASON_OPERATOR_START);
  } else {
    logLine("[CRSF] START ignored -- already flying/busy (phase: "
            + String(phaseName(phase)) + ")");
  }
}

#ifndef WOKWI_SIM
void crsfTask(void* parameter) {
  logLine("[CRSF] WARNING: CRSF_RX_PIN (GPIO" + String(CRSF_RX_PIN) + ") is a placeholder -- "
          "verify it doesn't collide with EspESC.hpp's pins before first flight.");
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
          crsfHandleStop();
        }

        // -- START: edge-triggered (LOW->HIGH transition only) --
        // Only triggers action when Ch5 crosses UP, so holding the switch does nothing extra.
        bool startHigh = (startVal > CRSF_HIGH_THRESHOLD);
        if (startHigh && !prevStartHigh) {
          crsfHandleStart();
        }
        prevStartHigh = startHigh;
      }

      bufLen = 0; // done with this frame, reset for next
    }
    vTaskDelay(pdMS_TO_TICKS(2)); // yield; 2 ms is well within the 4 ms frame interval
  }
}
#endif // !WOKWI_SIM

// ==========================================
// SETUP
// ==========================================
void setup() {
  sharedDataMutex = xSemaphoreCreateMutex(); 
  serialMutex     = xSemaphoreCreateMutex();
  
  Serial.begin(115200);
#ifdef WOKWI_SIM
  // ESP32-S3 native USB CDC takes a moment to enumerate after reset.
  // Without this delay, early Serial output is dropped before the host
  // sees the port. 2s is enough for macOS to reconnect and open the port.
  delay(2000);
#endif
#ifndef WOKWI_SIM
  Wire.begin(16, 15);
  initWiFi();
#else
  logLine("[WIFI] Sim mode — skipping WiFi, HIL runner talks over serial only.");
#endif

#ifndef WOKWI_SIM
  initLittleFS();
  initMPU();
  initCompass();
  initGPS();
  initESC();
#endif
#ifndef WOKWI_SIM
  barometer.initialize();
  
  // Average 20 barometer readings over ~2 s so the sensor has time to settle
  // and temperature effects are smoothed before we lock in the ground reference.
  logLine("[BARO] Sampling ground altitude (20 readings)...");
  {
    const int   BARO_SAMPLES     = 20;
    const int   BARO_INTERVAL_MS = 100;
    float accum = 0.0f;
    for (int i = 0; i < BARO_SAMPLES; i++) {
      accum += (float)barometer.readAltitudeMeters() * 3.28084f;
      delay(BARO_INTERVAL_MS);
    }
    groundAltitudeFt = accum / BARO_SAMPLES;
  }
  logLine(String("[BARO] Ground altitude locked: ") + String(groundAltitudeFt, 1) + " ft");
#else
  // In sim mode baroAltitudeFt is driven entirely by the HIL runner
  // via ALT: serial commands. No real sensor to read, no ground reference needed.
  groundAltitudeFt = 0.0f;
  logLine("[BARO] Sim mode -- barometer driven by HIL runner (ALT: commands).");
#endif

  xTaskCreatePinnedToCore(navigationTask, "NavTask",    8192, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(physicsTask,    "PhysicsTask", 8192, NULL, 2, NULL, 1);
#ifndef WOKWI_SIM
  // No real ELRS receiver exists in the Wokwi diagram, and CRSF_RX_PIN
  // currently collides with the I2C bus (see the pin-conflict note below).
  // Sim builds trigger START/STOP via parseSimInput() -> crsfHandleStart/Stop()
  // instead. TODO(hardware): CRSF_RX_PIN == 16 == Wire SDA. Move CRSF_RX_PIN
  // to a free GPIO once EspESC.hpp's pin usage is confirmed.
  xTaskCreatePinnedToCore(crsfTask,       "CRSFTask",    4096, NULL, 1, NULL, 0);
#endif

#ifndef WOKWI_SIM
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* r) { 
#ifdef WOKWI_SIM
    r->send(200, "text/plain", "Wokwi simulation");
#else
    r->send(LittleFS, "/index.html", "text/html"); 
#endif
  });
  
#ifndef WOKWI_SIM
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
#endif
  server.addHandler(&events);

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
    motors[motor - 1].write(throttle); // 0-indexed
    delay(2000);
    motors[motor - 1].disarm();

    logLine(String("[MOTOR TEST] M") + motor + " stopped.");
    r->send(200, "text/plain",
      String("M") + motor + " ran at " + pct + "% for 2 s — check spin direction in log");
  });

  server.begin();
  logLine("[WEB] Server started.");
#endif
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
#else
  // Was declared (lastGyroSend/lastAccSend/lastFlightSend, SSE_*_MS) but
  // never actually used anywhere -- the dashboard had no live data feed.
  unsigned long now = millis();
  if (now - lastGyroSend >= SSE_GYRO_MS) {
    events.send(getGyroReadings().c_str(), "gyro", now);
    lastGyroSend = now;
  }
  if (now - lastAccSend >= SSE_ACC_MS) {
    events.send(getAccReadings().c_str(), "acc", now);
    lastAccSend = now;
  }
  if (now - lastFlightSend >= SSE_FLIGHT_MS) {
    events.send(getFlightReadings().c_str(), "flight", now);
    lastFlightSend = now;
  }
#endif
}
