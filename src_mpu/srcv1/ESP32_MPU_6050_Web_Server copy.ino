/*********
  Rui Santos & Sara Santos - Random Nerd Tutorials
*********/
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
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

// NEW: Pure Physics Timer
unsigned long lastGyroMicros = 0; 

Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;

// Global variables holding our true state
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
// THE PHYSICS ENGINE (Runs as fast as possible)
// ==========================================
void updateIMU() {
  // 1. Calculate precise dt in seconds
  unsigned long currentMicros = micros();
  float dt = (currentMicros - lastGyroMicros) / 1000000.0;
  lastGyroMicros = currentMicros;

  // 2. Grab fresh data from the sensor
  mpu.getEvent(&a, &g, &temp);

  // 3. Pure Integration (NO DEADBAND)
  gyroX += g.gyro.x * dt;
  gyroY += g.gyro.y * dt;
  gyroZ += g.gyro.z * dt;

  // 4. Update the other globals while we are here
  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z;
  temperature = temp.temperature;
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
  // 1. UPDATE PHYSICS CONSTANTLY
  // This will run hundreds or thousands of times a second
  updateIMU(); 

  // 2. SEND DATA TO BROWSER ON A DELAY
  // This only happens when the timers pop
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