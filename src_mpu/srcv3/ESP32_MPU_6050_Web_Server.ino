#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
#include <MadgwickAHRS.h>
#include "LittleFS.h"

const char* ssid = "GossipGirl";
const char* password = "GossipGirl$1324";

AsyncWebServer server(80);
AsyncEventSource events("/events");
JsonDocument readings;

// Web server event timers
unsigned long lastTime = 0;
unsigned long lastTimeTemperature = 0;
unsigned long lastTimeAcc = 0;
unsigned long gyroDelay = 10;
unsigned long temperatureDelay = 1000;
unsigned long accelerometerDelay = 200;

Adafruit_MPU6050 mpu;
Madgwick filter;

// ==========================================
// SHARED STATE (written by Core 1, read by Core 0)
// Protected by sharedDataMutex
// ==========================================
SemaphoreHandle_t sharedDataMutex;

float sharedGyroX = 0, sharedGyroY = 0, sharedGyroZ = 0;
float sharedAccX = 0, sharedAccY = 0, sharedAccZ = 0;
float sharedTemperature = 0;
volatile bool resetRequested = false;

unsigned long lastGyroMicros = 0;

// ==========================================
// CORE 1: PHYSICS TASK (runs at ~200Hz)
// ==========================================
void physicsTask(void* parameter) {
  lastGyroMicros = micros();
  filter.begin(100); // initial guess, corrected every iteration below

  const TickType_t xFrequency = pdMS_TO_TICKS(5); // 200Hz -> ~5ms per tick
  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {
    // 1. Calculate precise dt in seconds
    unsigned long currentMicros = micros();
    float dt = (currentMicros - lastGyroMicros) / 1000000.0;
    lastGyroMicros = currentMicros;

    // 2. Read sensor (I2C - blocking, fine on this dedicated core)
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // 3. Convert gyro from rad/s to deg/s
    float gx = g.gyro.x * 57.2958;
    float gy = g.gyro.y * 57.2958;
    float gz = g.gyro.z * 57.2958;

    // 4. Tell the filter how much time actually passed
    if (dt > 0 && dt < 1.0) {
      filter.begin(1.0 / dt);
    }

    // 5. Run sensor fusion (6-axis: accel + gyro, no magnetometer)
    filter.updateIMU(gx, gy, gz, a.acceleration.x, a.acceleration.y, a.acceleration.z);

    // 6. Update shared state (protected by mutex)
    if (xSemaphoreTake(sharedDataMutex, portMAX_DELAY) == pdTRUE) {
      sharedGyroX = filter.getRoll();
      sharedGyroY = filter.getPitch();
      sharedGyroZ = filter.getYaw();
      sharedAccX = a.acceleration.x;
      sharedAccY = a.acceleration.y;
      sharedAccZ = a.acceleration.z;
      sharedTemperature = temp.temperature;
      xSemaphoreGive(sharedDataMutex);
    }

    // 7. Handle reset requests from Core 0
    if (resetRequested) {
      // Reset the internal quaternion back to "no rotation"
      filter.begin(100);  // this part is fine, resets timing
      // Then immediately re-feed it a "flat" reading
      // so it snaps the orientation estimate back to neutral
      filter.updateIMU(0, 0, 0, 0, 0, 1.0);  // stationary, gravity pointing down
      resetRequested = false;
    }

    // 8. Sleep until next 5ms tick (200Hz, precise pacing via FreeRTOS)
    vTaskDelayUntil(&lastWakeTime, xFrequency);
  }
}

// ==========================================
// WEB FORMATTERS (read from shared state)
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
  String jsonString;
  serializeJson(readings, jsonString);
  return jsonString;
}

String getAccReadings() {
  float ax, ay, az;
  if (xSemaphoreTake(sharedDataMutex, portMAX_DELAY) == pdTRUE) {
    ax = sharedAccX; ay = sharedAccY; az = sharedAccZ;
    xSemaphoreGive(sharedDataMutex);
  }
  readings["accX"] = ax;
  readings["accY"] = ay;
  readings["accZ"] = az;
  String accString;
  serializeJson(readings, accString);
  return accString;
}

String getTemperature() {
  float t;
  if (xSemaphoreTake(sharedDataMutex, portMAX_DELAY) == pdTRUE) {
    t = sharedTemperature;
    xSemaphoreGive(sharedDataMutex);
  }
  return String(t);
}

// ==========================================
// INIT HELPERS
// ==========================================
void initMPU(){
  Serial.println("Attempting to initialize MPU6050...");
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { delay(10); }
  }
  Serial.println("MPU6050 Found!");
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
  while (WiFi.status() != WL_CONNECTED) { delay(1000); }
  Serial.println(WiFi.localIP());
}

// ==========================================
// SETUP (runs on Core 0)
// ==========================================
void setup() {
  Serial.begin(115200);
  Wire.begin(16, 15);

  initWiFi();
  initLittleFS();
  initMPU();

  sharedDataMutex = xSemaphoreCreateMutex();

  // Launch physics loop on Core 1
  xTaskCreatePinnedToCore(
    physicsTask,
    "PhysicsTask",
    4096,    // stack size
    NULL,
    1,       // priority
    NULL,    // task handle (not needed)
    1        // Core 1
  );

  // Web Server Routes (Core 0)
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, "/index.html", "text/html");
  });

  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request){
    resetRequested = true;
    request->send(200, "text/plain", "OK");
  });

  events.onConnect([](AsyncEventSourceClient *client){
    client->send("hello!", NULL, millis(), 10000);
  });
  server.addHandler(&events);

  server.begin();
}

// ==========================================
// LOOP (Core 0, web server only)
// ==========================================
void loop() {
  if ((millis() - lastTime) > gyroDelay) {
    events.send(getGyroReadings().c_str(),"gyro_readings",millis());
    lastTime = millis();
  }
  if ((millis() - lastTimeAcc) > accelerometerDelay) {
    events.send(getAccReadings().c_str(),"accelerometer_readings",millis());
    lastTimeAcc = millis();
  }
  if ((millis() - lastTimeTemperature) > temperatureDelay) {
    events.send(getTemperature().c_str(),"temperature_reading",millis());
    lastTimeTemperature = millis();
  }
}
