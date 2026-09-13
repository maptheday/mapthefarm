#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
#include <MadgwickAHRS.h>
#include <TinyGPSPlus.h>

#include "hardware/EspBarometer.hpp"
#include "state/FlightConfig.hpp"
#include "models/FlightModel.hpp"
#include "models/SensorTypes.hpp"
#include "models/ControlTypes.hpp"
#include "state/PhaseState.hpp"
#include "state/HilState.hpp"
#include "services/Log.hpp"
#include "services/MotorController.hpp"
#include "services/Motors.hpp"
#include "phases/IFlightPhase.hpp"
#include "phases/PhaseRegistry.hpp"
#include "phases/PhaseMachine.hpp"   // transitionTo() + HIL gate
#include "services/Failsafes.hpp"    // checkCoreFailsafes()
#include "services/RcInput.hpp"      // CRSF radio + START/STOP handlers
#include "services/SimAdapter.hpp"   // HIL serial protocol (sim only)

// Pin/timing config lives in FlightConfig.hpp; sensors + motors live behind
// their services (Compass, Gps parsing in the nav task for now, Motors).

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

// The service instances (declared extern in FlightRuntime.hpp so phases can
// use them). Motors owns the 4 ESCs; Compass owns the magnetometer.
Motors          motors;
MotorController motorController;
Compass         compass;

// ==========================================
// The phase machine (transitionTo) + HIL gate live in phases/PhaseMachine.hpp.

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

// logLine() -> services/Log.hpp | GPS/nav math -> services/NavMath.hpp
// motor output -> services/Motors.hpp | phase machine -> phases/PhaseMachine.hpp
// core failsafes -> services/Failsafes.hpp

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
    float heading = compass.readHeadingDeg();
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

// Compass bring-up + calibration now live in the Compass service; live
// calibration is the CalibratePhase (triggered on boot below if configured).

void initGPS() {
  Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
}


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
  initMPU();
  compass.begin();
  logLine("[COMPASS] QMC5883L ready.");
  initGPS();
  motors.begin();
  logLine("[ESC] DShot600 channels initialized, all motors disarmed.");
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

  // Optional compass calibration on boot. It's a normal phase now: the nav/
  // physics tasks (just started) run CalibratePhase, which returns to PARKED
  // when done. Motors stay disarmed throughout.
  if (CALIBRATE_COMPASS_ON_BOOT) {
    transitionTo(PHASE_CALIBRATE);
  }
#endif
}

// ==========================================
// LOOP (Core 0)
// ==========================================
void loop() {
#ifdef WOKWI_SIM
  // In sim, the HIL runner drives the drone over serial.
  parseSimInput();
#else
  // On real hardware everything runs in the nav/physics/CRSF tasks;
  // the Arduino loop has nothing to do.
  vTaskDelay(pdMS_TO_TICKS(100));
#endif
}
