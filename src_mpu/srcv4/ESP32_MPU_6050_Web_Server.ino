#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
#include <MadgwickAHRS.h>
#include "LittleFS.h"

// Local hardware drivers (same folder as this .ino)
#include "EspBarometer.hpp"
#include "EspESC.hpp"

const char* ssid     = "GossipGirl";
const char* password = "GossipGirl$1324";

AsyncWebServer server(80);
AsyncEventSource events("/events");
JsonDocument readings;

// ==========================================
// HARDWARE INSTANCES
// ==========================================
Adafruit_MPU6050 mpu;
Madgwick filter;
EspBarometer barometer;
EspESC esc;

// ==========================================
// SHARED STATE
// Written by Core 1 (physics), read by Core 0 (web server)
// All access protected by sharedDataMutex
// ==========================================
SemaphoreHandle_t sharedDataMutex;

// IMU
float sharedGyroX    = 0, sharedGyroY    = 0, sharedGyroZ    = 0;
float sharedAccX     = 0, sharedAccY     = 0, sharedAccZ     = 0;
float sharedTemperature = 0;

// Barometer
float sharedAltitude = 0;

// Motors (0.0 - 1.0)
float sharedM1 = 0, sharedM2 = 0, sharedM3 = 0, sharedM4 = 0;

// Flags written by Core 0, consumed by Core 1
volatile bool resetRequested   = false;
volatile bool motorsArmed      = false;
volatile float manualThrottle  = 0.0;  // 0.0 - 1.0, used when armed

// ==========================================
// CORE 1: PHYSICS TASK (~200Hz)
// Handles: IMU, Madgwick filter, barometer, ESC writes
// ==========================================
unsigned long lastGyroMicros = 0;

void physicsTask(void* parameter) {
  lastGyroMicros = micros();
  filter.begin(100); // initial guess; corrected each loop via real dt

  const TickType_t xFrequency = pdMS_TO_TICKS(5); // 200Hz = every 5ms
  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {
    // 1. Measure real dt since last iteration
    unsigned long currentMicros = micros();
    float dt = (currentMicros - lastGyroMicros) / 1000000.0;
    lastGyroMicros = currentMicros;

    // 2. Read IMU (I2C blocking — fine on dedicated core)
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // 3. Convert gyro rad/s → deg/s (Madgwick expects deg/s)
    float gx = g.gyro.x * 57.2958;
    float gy = g.gyro.y * 57.2958;
    float gz = g.gyro.z * 57.2958;

    // 4. Tell the filter how much time actually passed this loop
    if (dt > 0 && dt < 1.0) {
      filter.begin(1.0 / dt);
    }

    // 5. Sensor fusion: accel corrects gyro drift over time
    filter.updateIMU(gx, gy, gz, a.acceleration.x, a.acceleration.y, a.acceleration.z);

    // 6. Read altitude from barometer
    float altitude = (float)barometer.readAltitudeMeters();

    // 7. Motor control
    //    Armed: send manual throttle to all 4 motors equally (no PID yet)
    //    Disarmed: send 0 to all motors
    float m1 = 0, m2 = 0, m3 = 0, m4 = 0;
    if (motorsArmed) {
      m1 = m2 = m3 = m4 = manualThrottle;
      esc.write(m1, m2, m3, m4);
    } else {
      esc.disarm();
    }

    // 8. Handle reset request from web UI
    if (resetRequested) {
      filter.begin(100);
      filter.updateIMU(0, 0, 0, 0, 0, 1.0); // snap orientation back to flat
      resetRequested = false;
    }

    // 9. Write all results to shared state (mutex protected)
    if (xSemaphoreTake(sharedDataMutex, portMAX_DELAY) == pdTRUE) {
      sharedGyroX       = filter.getRoll();
      sharedGyroY       = filter.getPitch();
      sharedGyroZ       = filter.getYaw();
      sharedAccX        = a.acceleration.x;
      sharedAccY        = a.acceleration.y;
      sharedAccZ        = a.acceleration.z;
      sharedTemperature = temp.temperature;
      sharedAltitude    = altitude;
      sharedM1 = m1; sharedM2 = m2; sharedM3 = m3; sharedM4 = m4;
      xSemaphoreGive(sharedDataMutex);
    }

    // 10. Sleep until next 5ms tick (FreeRTOS handles precise pacing)
    vTaskDelayUntil(&lastWakeTime, xFrequency);
  }
}

// ==========================================
// WEB FORMATTERS
// All read from shared state via mutex
// ==========================================
String getGyroReadings() {
  float gx, gy, gz;
  if (xSemaphoreTake(sharedDataMutex, portMAX_DELAY) == pdTRUE) {
    gx = sharedGyroX; gy = sharedGyroY; gz = sharedGyroZ;
    xSemaphoreGive(sharedDataMutex);
  }
  readings["gyroX"] = gx;
  readings["gyroY"] = gy;
  readings["gyroZ"] = gz;
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

String getTemperature() {
  float t;
  if (xSemaphoreTake(sharedDataMutex, portMAX_DELAY) == pdTRUE) {
    t = sharedTemperature;
    xSemaphoreGive(sharedDataMutex);
  }
  return String(t);
}

String getAltitude() {
  float alt;
  if (xSemaphoreTake(sharedDataMutex, portMAX_DELAY) == pdTRUE) {
    alt = sharedAltitude;
    xSemaphoreGive(sharedDataMutex);
  }
  return String(alt, 2); // 2 decimal places
}

String getMotorReadings() {
  float m1, m2, m3, m4;
  if (xSemaphoreTake(sharedDataMutex, portMAX_DELAY) == pdTRUE) {
    m1 = sharedM1; m2 = sharedM2; m3 = sharedM3; m4 = sharedM4;
    xSemaphoreGive(sharedDataMutex);
  }
  readings["m1"] = (int)(m1 * 100);
  readings["m2"] = (int)(m2 * 100);
  readings["m3"] = (int)(m3 * 100);
  readings["m4"] = (int)(m4 * 100);
  readings["armed"] = motorsArmed;
  String out; serializeJson(readings, out); return out;
}

// ==========================================
// INIT HELPERS
// ==========================================
void initMPU() {
  Serial.println("[IMU] Initializing MPU6050...");
  if (!mpu.begin()) {
    Serial.println("[IMU] ERROR: MPU6050 not found. Check wiring.");
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
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000); Serial.print(".");
  }
  Serial.println();
  Serial.print("[WiFi] Connected: ");
  Serial.println(WiFi.localIP());
}

// ==========================================
// SETUP (runs on Core 0)
// ==========================================
void setup() {
  Serial.begin(115200);

  // Single shared I2C bus for both MPU6050 and BME280
  // MPU6050: address 0x68, BME280: address 0x76 — no conflict
  Wire.begin(16, 15); // SDA=16, SCL=15

  initWiFi();
  initLittleFS();
  initMPU();

  // Barometer: strips its own Wire.begin() since we already called it above
  barometer.initialize();

  // ESC: arms all 4 motors, sends 0 throttle handshake
  esc.initialize();

  // Create mutex before launching physics task
  sharedDataMutex = xSemaphoreCreateMutex();

  // Launch physics loop on Core 1
  xTaskCreatePinnedToCore(
    physicsTask,
    "PhysicsTask",
    8192,   // stack size (larger than before — barometer math needs it)
    NULL,
    1,      // priority
    NULL,
    1       // Core 1
  );

  // ── Web Server Routes ──────────────────────────────────────

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(LittleFS, "/index.html", "text/html");
  });

  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  // Reset orientation
  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest* request) {
    resetRequested = true;
    request->send(200, "text/plain", "OK");
  });

  // Arm motors (enables ESC output)
  server.on("/arm", HTTP_GET, [](AsyncWebServerRequest* request) {
    motorsArmed = true;
    Serial.println("[ESC] Motors ARMED");
    request->send(200, "text/plain", "ARMED");
  });

  // Disarm motors (cuts ESC output immediately)
  server.on("/disarm", HTTP_GET, [](AsyncWebServerRequest* request) {
    motorsArmed = false;
    manualThrottle = 0.0;
    Serial.println("[ESC] Motors DISARMED");
    request->send(200, "text/plain", "DISARMED");
  });

  // Set throttle: /throttle?value=0.25  (0.0 - 1.0)
  server.on("/throttle", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (request->hasParam("value")) {
      float val = request->getParam("value")->value().toFloat();
      if (val < 0.0) val = 0.0;
      if (val > 1.0) val = 1.0;
      manualThrottle = val;
      request->send(200, "text/plain", "OK");
    } else {
      request->send(400, "text/plain", "Missing value param");
    }
  });

  // SSE events
  events.onConnect([](AsyncEventSourceClient* client) {
    client->send("hello!", NULL, millis(), 10000);
  });
  server.addHandler(&events);

  server.begin();
  Serial.println("[Web] Server started.");
}

// ==========================================
// LOOP (Core 0 — web server + SSE dispatch only)
// ==========================================
unsigned long lastTime            = 0;
unsigned long lastTimeTemperature = 0;
unsigned long lastTimeAcc         = 0;
unsigned long lastTimeAltitude    = 0;
unsigned long lastTimeMotors      = 0;

unsigned long gyroDelay          = 10;   // 100Hz to browser
unsigned long temperatureDelay   = 1000; // 1Hz
unsigned long accelerometerDelay = 200;  // 5Hz
unsigned long altitudeDelay      = 200;  // 5Hz
unsigned long motorDelay         = 100;  // 10Hz

void loop() {
  if ((millis() - lastTime) > gyroDelay) {
    events.send(getGyroReadings().c_str(), "gyro_readings", millis());
    lastTime = millis();
  }
  if ((millis() - lastTimeAcc) > accelerometerDelay) {
    events.send(getAccReadings().c_str(), "accelerometer_readings", millis());
    lastTimeAcc = millis();
  }
  if ((millis() - lastTimeTemperature) > temperatureDelay) {
    events.send(getTemperature().c_str(), "temperature_reading", millis());
    lastTimeTemperature = millis();
  }
  if ((millis() - lastTimeAltitude) > altitudeDelay) {
    events.send(getAltitude().c_str(), "altitude_reading", millis());
    lastTimeAltitude = millis();
  }
  if ((millis() - lastTimeMotors) > motorDelay) {
    events.send(getMotorReadings().c_str(), "motor_readings", millis());
    lastTimeMotors = millis();
  }
}
