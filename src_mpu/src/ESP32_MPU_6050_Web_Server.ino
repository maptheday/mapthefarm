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

const char* ssid     = "Wokwi-GUEST";
const char* password = "";

// ==========================================
// TIMING CONFIGURATION
// Increase these values to slow down the loops.
// ==========================================
// change this back to 5ms for 200Hz physics loop once testing is done
const unsigned long PHYSICS_LOOP_MS = 5;  // 10000ms = 10s
const float PHYSICS_LOOP_HZ = 1000.0f / PHYSICS_LOOP_MS;
const unsigned long SSE_GYRO_MS    = 10;  // orientation updates
const unsigned long SSE_ACC_MS     = 200; // accelerometer updates
const unsigned long SSE_FLIGHT_MS  = 100; // flight/state updates

AsyncWebServer server(80);
AsyncEventSource events("/events");
JsonDocument readings;

// ==========================================
// HARDWARE
// ==========================================
Adafruit_MPU6050 mpu;
Madgwick filter;
EspBarometer barometer;
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
// For yaw, the correction is gentler than roll/pitch because spinning the whole drone is slower and more sluggish than tilting it:
// ==========================================
PID altitudePID(0.08f, 0.01f, 0.05f, 0.0f,  1.0f);
PID rollPID    (0.01f, 0.001f, 0.005f, -0.3f, 0.3f);
PID pitchPID   (0.01f, 0.001f, 0.005f, -0.3f, 0.3f);
PID yawPID(0.005f, 0.0001f, 0.001f, -0.2f, 0.2f);

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
volatile bool  flightEnabled   = true; // true = autonomous flight active
volatile float targetAltFt     = 10.0f;  // feet, set from web UI
// volatile bool  resetIMURequest = false;

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
      // filter.begin(1.0f / dt);
      filter.updateIMU(gx, gy, gz, a.acceleration.x, a.acceleration.y, a.acceleration.z);
    }

    // filter.updateIMU(gx, gy, gz, a.acceleration.x, a.acceleration.y, a.acceleration.z);

    float currentRoll  = filter.getRoll();
    float currentPitch = filter.getPitch();
    float currentYaw   = filter.getYaw();

    // ── 3. Read altitude (meters → feet) ───────────────────
    float altitudeFt = (float)barometer.readAltitudeMeters() * 3.28084f;

    // Log pitch and altitude each physics cycle
    // Serial.print("[PHYSICS] pitch=");
    // Serial.print(currentPitch, 2);
    // Serial.print(" deg, altFt=");
    // Serial.print(altitudeFt, 2);
    // Serial.print(" ft, dt=");
    // Serial.print(dt, 4);
    // Serial.println(" s");

    // ── 4. IMU reset request ────────────────────────────────
    // if (resetIMURequest) {
    //   filter.begin(100);
    //   filter.updateIMU(0, 0, 0, 0, 0, 1.0f);
    //   altitudePID.reset();
    //   rollPID.reset();
    //   pitchPID.reset();
    //   resetIMURequest = false;
    // }

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
      // "Tilting right? Spin left motors faster to level out."
      rollCorrection = rollPID.compute(0.0f, currentRoll, dt);

      // Pitch PID → front/rear motor trim
      // "Nose down? Spin rear motors faster to level out."
      pitchCorrection = pitchPID.compute(0.0f, currentPitch, dt);


      // Wrap yaw so the PID sees the shortest path to target
      // Yaw PID → diagonal motor trim to rotate drone back to target heading
      // P — how far from north right now?
      //     "90 degrees off → spin hard"
      // I — _integral accumulates (error × dt) every loop
      //     grows larger the longer we stay off heading
      //     "been drifting west for a while, probably wind → spin a bit harder"
      // D — how fast is the error shrinking?
      //     "already rotating back to north fast → ease off before we overshoot east"
      //
      // output: single float fed into motor mixer to spin the drone left or right
      float targetHeading = 0.0f; // true north, or whatever heading you want to maintain
      float yawError = targetHeading - currentYaw;
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
      // esc1.disarm(); esc2.disarm(); esc3.disarm(); esc4.disarm();
    }

    Serial.print("[PHYSICS] pitch=");
    Serial.print(currentPitch, 2);
    Serial.print(" deg, altFt=");
    Serial.print(altitudeFt, 2);
    Serial.print(" ft, dt=");
    Serial.print(dt, 4);
    Serial.print(" s, rollCorr=");
    Serial.print(sharedRollCorrection, 3);
    Serial.print(" pitchCorr=");
    Serial.print(sharedPitchCorrection, 3);
    Serial.print(" m1=");
    Serial.print(m1, 2);
    Serial.print(" m2=");
    Serial.print(m2, 2);
    Serial.print(" m3=");
    Serial.print(m3, 2);
    Serial.print(" m4=");
    Serial.println(m4, 2);

    // ── 6. Write to shared state ────────────────────────────
    if (xSemaphoreTake(sharedDataMutex, portMAX_DELAY) == pdTRUE) {
      sharedRoll  = currentRoll;
      sharedPitch = currentPitch;
      sharedYaw   = currentYaw;
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

  Serial.println("=== BOOT START ===");

  initWiFi();
  initLittleFS();
  initMPU();
  barometer.initialize();
  // esc1.init(4, RMT_CHANNEL_0); // M1 front-left
  // esc2.init(5, RMT_CHANNEL_1); // M2 front-right
  // esc3.init(6, RMT_CHANNEL_2); // M3 rear-left
  // esc4.init(7, RMT_CHANNEL_3); // M4 rear-right
  Serial.println("[ESC] DShot600 ready on GPIO 4/5/6/7");

  sharedDataMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(physicsTask, "PhysicsTask", 8192, NULL, 1, NULL, 1);

  // ── Routes ──────────────────────────────────────────────

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* r) {
    r->send(LittleFS, "/index.html", "text/html");
  });

  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  // Reset IMU orientation
  // server.on("/reset", HTTP_GET, [](AsyncWebServerRequest* r) {
  //   resetIMURequest = true;
  //   r->send(200, "text/plain", "OK");
  // });

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
  // Flight data: altitude, motors, PID debug
  if (millis() - lastFlightSend > SSE_FLIGHT_MS) {
    events.send(getFlightReadings().c_str(), "flight_readings", millis());
    lastFlightSend = millis();
  }
}
