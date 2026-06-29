#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
#include <MadgwickAHRS.h>
#include "LittleFS.h"

#include "EspBarometer.hpp"
#include "EspESC.hpp"
#include "PID.hpp"

const char* ssid     = "GossipGirl";
const char* password = "GossipGirl$1324";

AsyncWebServer server(80);
AsyncEventSource events("/events");
JsonDocument readings;

// ==========================================
// HARDWARE
// ==========================================
Adafruit_MPU6050 mpu;
Madgwick filter;
EspBarometer barometer;
EspESC esc;

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
// ==========================================
PID altitudePID(0.08f, 0.01f, 0.05f, 0.0f,  1.0f);
PID rollPID    (0.01f, 0.001f, 0.005f, -0.3f, 0.3f);
PID pitchPID   (0.01f, 0.001f, 0.005f, -0.3f, 0.3f);

// ==========================================
// SHARED STATE
// Written by Core 1, read by Core 0
// ==========================================
SemaphoreHandle_t sharedDataMutex;

// IMU
float sharedRoll  = 0, sharedPitch = 0, sharedYaw = 0;
float sharedAccX  = 0, sharedAccY  = 0, sharedAccZ = 0;
float sharedTemp  = 0;

// Barometer
float sharedAltitudeFt = 0; // feet

// Motors (0.0 - 1.0, sent as 0-100 to UI)
float sharedM1 = 0, sharedM2 = 0, sharedM3 = 0, sharedM4 = 0;

// PID debug (so UI can show what each loop is doing)
float sharedBaseThrottle   = 0;
float sharedRollCorrection = 0;
float sharedPitchCorrection = 0;

// Flags / commands (written by Core 0, consumed by Core 1)
volatile bool  flightEnabled   = false; // true = autonomous flight active
volatile float targetAltFt     = 0.0f;  // feet, set from web UI
volatile bool  resetIMURequest = false;

// ==========================================
// CORE 1: PHYSICS + FLIGHT TASK (~200Hz)
// ==========================================
unsigned long lastGyroMicros = 0;

void physicsTask(void* parameter) {
  lastGyroMicros = micros();
  filter.begin(100);

  const TickType_t xFrequency = pdMS_TO_TICKS(5); // 200Hz
  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {

    // ── 1. Measure real dt ──────────────────────────────────
    unsigned long now = micros();
    float dt = (now - lastGyroMicros) / 1000000.0f;
    lastGyroMicros = now;

    // ── 2. Read IMU ─────────────────────────────────────────
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Convert gyro from rad/s → deg/s
    // The MPU6050 gives us rotational speed in radians per second
    // but Madgwick expects degrees per second — 57.2958 is just the conversion factor
    float gx = g.gyro.x * 57.2958f;
    float gy = g.gyro.y * 57.2958f;
    float gz = g.gyro.z * 57.2958f;

    // Tell the filter how much time actually passed since the last loop
    // This is the "ruler" — without it, the filter can't convert
    // "spinning at 20°/sec" into "rotated X degrees"
    // (the old API wants frequency not time, so we flip it: 1/dt)
    if (dt > 0 && dt < 1.0f) filter.begin(1.0f / dt);

    // Feed gyro + accelerometer into the filter
    // Gyro does the dead reckoning: rotation += speed × dt
    // Accelerometer corrects slow drift by always knowing which way is "down"
    // Output is a best-guess orientation that gets more accurate over time
    filter.updateIMU(gx, gy, gz, a.acceleration.x, a.acceleration.y, a.acceleration.z);

    // Pull the drift-corrected orientation out of the filter (degrees)
    // Roll  = tilt left/right
    // Pitch = tilt forward/backward
    // Yaw   = spin left/right (drifts over time — no magnetometer to anchor it)
    float roll  = filter.getRoll();
    float pitch = filter.getPitch();
    float yaw   = filter.getYaw();

    // ── 3. Read altitude (meters → feet) ───────────────────
    float altitudeFt = (float)barometer.readAltitudeMeters() * 3.28084f;

    // ── 4. IMU reset request ────────────────────────────────
    if (resetIMURequest) {
      filter.begin(100);
      filter.updateIMU(0, 0, 0, 0, 0, 1.0f);
      altitudePID.reset();
      rollPID.reset();
      pitchPID.reset();
      resetIMURequest = false;
    }

    // ── 5. Flight control ───────────────────────────────────
    float m1 = 0, m2 = 0, m3 = 0, m4 = 0;
    float baseThrottle    = 0;
    float rollCorrection  = 0;
    float pitchCorrection = 0;

    if (flightEnabled) {
      // Altitude PID → base throttle for all motors
      // "How far from target? Spin all motors faster/slower."
      baseThrottle = altitudePID.compute(targetAltFt, altitudeFt, dt);

      // Roll PID → left/right motor trim
      // "Tilting right? Spin left motors faster to level out."
      rollCorrection = rollPID.compute(0.0f, roll, dt);

      // Pitch PID → front/rear motor trim
      // "Nose down? Spin rear motors faster to level out."
      pitchCorrection = pitchPID.compute(0.0f, pitch, dt);

      // Motor mixer: combine base throttle + stabilization corrections
      //
      //      FRONT
      //  M1 (FL)   M2 (FR)
      //  M3 (RL)   M4 (RR)
      //      REAR
      //
      // zach: rollcorrection is positive when the drone is tiling right 
      // so on the right motors we subtract it and on the left motors we add it. 
      // Pitch correction is positive when the nose is up
      // so on the front motors we add it and on the rear motors we subtract it.
      //
      // Roll +  → tilt right → FL & RL need more power (left side lifts)
      // Pitch + → nose up    → FL & FR need more power (front lifts)
      m1 = baseThrottle + pitchCorrection + rollCorrection;  // FL
      m2 = baseThrottle + pitchCorrection - rollCorrection;  // FR
      m3 = baseThrottle - pitchCorrection + rollCorrection;  // RL
      m4 = baseThrottle - pitchCorrection - rollCorrection;  // RR

      // Clamp all motors to valid range
      m1 = constrain(m1, 0.0f, 1.0f);
      m2 = constrain(m2, 0.0f, 1.0f);
      m3 = constrain(m3, 0.0f, 1.0f);
      m4 = constrain(m4, 0.0f, 1.0f);

      esc.write(m1, m2, m3, m4);
    } else {
      // Flight disabled — all motors off
      altitudePID.reset();
      rollPID.reset();
      pitchPID.reset();
      esc.disarm();
    }

    // ── 6. Write to shared state ────────────────────────────
    if (xSemaphoreTake(sharedDataMutex, portMAX_DELAY) == pdTRUE) {
      sharedRoll  = roll;
      sharedPitch = pitch;
      sharedYaw   = yaw;
      sharedAccX  = a.acceleration.x;
      sharedAccY  = a.acceleration.y;
      sharedAccZ  = a.acceleration.z;
      sharedTemp  = temp.temperature;
      sharedAltitudeFt     = altitudeFt;
      sharedM1 = m1; sharedM2 = m2; sharedM3 = m3; sharedM4 = m4;
      sharedBaseThrottle    = baseThrottle;
      sharedRollCorrection  = rollCorrection;
      sharedPitchCorrection = pitchCorrection;
      xSemaphoreGive(sharedDataMutex);
    }

    vTaskDelayUntil(&lastWakeTime, xFrequency);
  }
}

// ==========================================
// WEB FORMATTERS
// ==========================================
String getGyroReadings() {
  float r, p, y;
  if (xSemaphoreTake(sharedDataMutex, portMAX_DELAY) == pdTRUE) {
    r = sharedRoll; p = sharedPitch; y = sharedYaw;
    xSemaphoreGive(sharedDataMutex);
  }
  readings["gyroX"] = r;
  readings["gyroY"] = p;
  readings["gyroZ"] = y;
  String out; serializeJson(readings, out); return out;
}

String getAccReadings() {
  float ax, ay, az;
  if (xSemaphoreTake(sharedDataMutex, portMAX_DELAY) == pdTRUE) {
    ax = sharedAccX; ay = sharedAccY; az = sharedAccZ;
    xSemaphoreGive(sharedDataMutex);
  }
  readings["accX"] = ax; readings["accY"] = ay; readings["accZ"] = az;
  String out; serializeJson(readings, out); return out;
}

String getFlightReadings() {
  float alt, m1, m2, m3, m4, base, roll, pitch;
  if (xSemaphoreTake(sharedDataMutex, portMAX_DELAY) == pdTRUE) {
    alt   = sharedAltitudeFt;
    m1    = sharedM1; m2 = sharedM2; m3 = sharedM3; m4 = sharedM4;
    base  = sharedBaseThrottle;
    roll  = sharedRollCorrection;
    pitch = sharedPitchCorrection;
    xSemaphoreGive(sharedDataMutex);
  }
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
  String out; serializeJson(readings, out); return out;
}

// ==========================================
// INIT HELPERS
// ==========================================
void initMPU() {
  Serial.println("[IMU] Initializing MPU6050...");
  if (!mpu.begin()) {
    Serial.println("[IMU] ERROR: MPU6050 not found.");
    while (1) { delay(10); }
  }
  Serial.println("[IMU] MPU6050 ready.");
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
  Serial.print("[WiFi] Connecting");
  while (WiFi.status() != WL_CONNECTED) { delay(1000); Serial.print("."); }
  Serial.println();
  Serial.print("[WiFi] ");
  Serial.println(WiFi.localIP());
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  Wire.begin(16, 15); // SDA=16, SCL=15 — shared I2C bus for MPU6050 + BME280

  initWiFi();
  initLittleFS();
  initMPU();
  barometer.initialize();
  esc.initialize();

  sharedDataMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(physicsTask, "PhysicsTask", 8192, NULL, 1, NULL, 1);

  // ── Routes ──────────────────────────────────────────────

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* r) {
    r->send(LittleFS, "/index.html", "text/html");
  });

  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  // Reset IMU orientation
  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest* r) {
    resetIMURequest = true;
    r->send(200, "text/plain", "OK");
  });

  // Enable autonomous flight with target altitude in feet
  // e.g. /fly?ft=10
  server.on("/fly", HTTP_GET, [](AsyncWebServerRequest* r) {
    if (r->hasParam("ft")) {
      float ft = r->getParam("ft")->value().toFloat();
      if (ft < 0)    ft = 0;
      if (ft > 50)   ft = 50; // safety cap at 50ft
      targetAltFt   = ft;
      flightEnabled = true;
      Serial.print("[FLIGHT] Target: ");
      Serial.print(ft); Serial.println(" ft");
    }
    r->send(200, "text/plain", "OK");
  });

  // Land: target drops to 0, flight disables once landed
  server.on("/land", HTTP_GET, [](AsyncWebServerRequest* r) {
    targetAltFt   = 0.0f;
    // Flight stays enabled so PID can descend gently to 0ft
    // Core 1 will disarm once altitude is near 0
    flightEnabled = true;
    r->send(200, "text/plain", "LANDING");
  });

  // Emergency stop — cuts motors immediately
  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest* r) {
    flightEnabled = false;
    targetAltFt   = 0.0f;
    r->send(200, "text/plain", "STOPPED");
  });

  events.onConnect([](AsyncEventSourceClient* c) {
    c->send("hello!", NULL, millis(), 10000);
  });
  server.addHandler(&events);
  server.begin();

  Serial.println("[Web] Server started.");
}

// ==========================================
// LOOP (Core 0 — SSE dispatch only)
// ==========================================
unsigned long lastGyroSend   = 0;
unsigned long lastAccSend    = 0;
unsigned long lastFlightSend = 0;

void loop() {
  // Orientation → cube rotation (100Hz)
  if (millis() - lastGyroSend > 10) {
    events.send(getGyroReadings().c_str(), "gyro_readings", millis());
    lastGyroSend = millis();
  }
  // Accelerometer (5Hz)
  if (millis() - lastAccSend > 200) {
    events.send(getAccReadings().c_str(), "accelerometer_readings", millis());
    lastAccSend = millis();
  }
  // Flight data: altitude, motors, PID debug (10Hz)
  if (millis() - lastFlightSend > 100) {
    events.send(getFlightReadings().c_str(), "flight_readings", millis());
    lastFlightSend = millis();
  }
}
