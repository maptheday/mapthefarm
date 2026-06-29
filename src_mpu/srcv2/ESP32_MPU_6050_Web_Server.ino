#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
#include <MadgwickAHRS.h>
#include "LittleFS.h"

const char* ssid = "Test";
const char* password = "Test$1324";

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

unsigned long lastPhysicsMicros = 0;
const unsigned long physicsIntervalMicros = 5000; // 200Hz

// Pure Physics Timer
unsigned long lastGyroMicros = 0; 

Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;

// Madgwick filter instance
Madgwick filter;

// Global variables holding our true state
// gyroX/Y/Z now hold the FUSED, drift-corrected orientation (roll/pitch/yaw, in degrees)
float gyroX = 0, gyroY = 0, gyroZ = 0;
float accX = 0, accY = 0, accZ = 0;
float temperature = 0;

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
// THE WEB FORMATTERS (Just string packages)
// ==========================================
String getGyroReadings() {
  readings["gyroX"] = gyroX;
  readings["gyroY"] = gyroY;
  readings["gyroZ"] = gyroZ;
  String jsonString;
  serializeJson(readings, jsonString);
  return jsonString;
}

String getAccReadings() {
  readings["accX"] = accX;
  readings["accY"] = accY;
  readings["accZ"] = accZ;
  String accString;
  serializeJson(readings, accString);
  return accString;
}

String getTemperature() {
  return String(temperature);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(16, 15);
  
  initWiFi();
  initLittleFS();
  initMPU();

  // Initialize the physics stopwatch
  lastGyroMicros = micros();

  // Initialize the Madgwick filter with a default sample rate;
  // it gets updated each loop in updateIMU() based on real dt
  filter.begin(100);

  // Web Server Routes
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, "/index.html", "text/html");
  });
  
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request){
    gyroX=0; gyroY=0; gyroZ=0;
    request->send(200, "text/plain", "OK");
  });

  events.onConnect([](AsyncEventSourceClient *client){
    client->send("hello!", NULL, millis(), 10000);
  });
  server.addHandler(&events);

  server.begin();
}

void loop() {
  // The real, simple reason is Thing 1: there's no point sampling faster than the sensor can produce new data. 
  // 200Hz vs 2000Hz won't make your tracking meaningfully better, 
  // because past a certain point you're just reading stale numbers and burning CPU that the web server also needs. 
  if (micros() - lastPhysicsMicros >= physicsIntervalMicros) {

    lastPhysicsMicros = micros();

    // 1. Calculate precise dt in seconds
    unsigned long currentMicros = micros();
    float dt = (currentMicros - lastGyroMicros) / 1000000.0;
    lastGyroMicros = currentMicros;

    // 2. Grab fresh data from the sensor
    mpu.getEvent(&a, &g, &temp);

    // 3. Update other globals (raw accelerometer values, for display/debug)
    accX = a.acceleration.x;
    accY = a.acceleration.y;
    accZ = a.acceleration.z;
    temperature = temp.temperature;

    // 4. Madgwick filter setup expects:
    //    - gyro in degrees/second
    //    - accel in any consistent units (the filter normalizes internally)
    //    Adafruit_MPU6050 returns gyro in rad/s, so convert to deg/s.
    float gx = g.gyro.x * 57.2958; // rad/s -> deg/s
    float gy = g.gyro.y * 57.2958;
    float gz = g.gyro.z * 57.2958;

    // This just tells the filter "hey, last loop took this long." That's it. 
    // It's setting up the ruler the filter will use to convert speed into rotation amount.
    if (dt > 0) {
      filter.begin(1.0 / dt);
    }

    // 6. Run sensor fusion (6-axis: accel + gyro, no magnetometer)
    filter.updateIMU(gx, gy, gz, accX, accY, accZ);

    // 7. Pull the fused, drift-corrected orientation (in degrees)
    gyroX = filter.getRoll();
    gyroY = filter.getPitch();
    gyroZ = filter.getYaw();
    // updateIMU();
  }

  // 2. SEND DATA TO BROWSER ON A DELAY
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