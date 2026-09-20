#include <Arduino.h>

#include "state/FlightConfig.hpp"
#include "models/FlightModel.hpp"
#include "models/SensorTypes.hpp"
#include "models/ControlTypes.hpp"
#include "state/PhaseState.hpp"
#include "services/Log.hpp"
#include "services/MotorController.hpp"
#include "services/Motors.hpp"
#include "services/Compass.hpp"       // magnetometer
#include "services/Gps.hpp"           // GPS receiver
#include "services/Imu.hpp"           // accel/gyro + attitude filter
#include "services/Altimeter.hpp"     // barometer height-above-ground
#include "phases/IFlightPhase.hpp"
#include "phases/PhaseRegistry.hpp"
#include "phases/PhaseMachine.hpp"   // transitionTo()
#include "services/Failsafes.hpp"    // checkCoreFailsafes()
#include "services/RcInput.hpp"      // CRSF radio + START/STOP handlers
#include "services/OnboardSim.hpp"   // on-chip physics simulation (SIM only)
#include "services/SimAdapter.hpp"   // DUMPLOG serial command (SIM only)

// Pin/timing config lives in FlightConfig.hpp; every sensor and actuator lives
// behind its own service (Imu, Altimeter, Gps, Compass, Motors).

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
// SERVICE INSTANCES
// ------------------------------------------
// The one real object for each service. Every service header declares these
// `extern` so any file can reach the same one; they are born here, once (this
// .ino is the composition root).
// ==========================================
Motors          motors;          // the 4 ESCs
MotorController motorController;  // PID + motor mixing
Compass         compass;         // magnetometer + calibration
Gps             gps;             // GPS receiver
Imu             imu;             // accel/gyro + attitude filter
Altimeter       altimeter;       // barometer height-above-ground

// ==========================================
// The phase machine (transitionTo) lives in phases/PhaseMachine.hpp.

// ==========================================
// SHARED STATE (repository)
// ------------------------------------------
// Per-phase state blocks, SharedState, and withMutex() live in PhaseState.hpp.
// The single instances are DEFINED here (the .ino owns them); PhaseState.hpp
// declares `shared` + the mutex `extern` for the rest of the code.
// ==========================================
SemaphoreHandle_t sharedDataMutex;
SemaphoreHandle_t serialMutex;

volatile SharedState shared;

// ==========================================
// TASKS
// ==========================================
void navigationTask(void* parameter) {
  const TickType_t xFrequency = pdMS_TO_TICKS(NAV_LOOP_MS);
  TickType_t lastWakeTime     = xTaskGetTickCount();
  const float navDt           = NAV_LOOP_MS / 1000.0f;

  for (;;) {
#if defined(SIM)
    // On-chip sim: QuadSim wrote GPS in the physics tick. Here we just run the
    // autonomous mission driver, heading feed, and flight logging.
    onboardSimNav();
#else
    RawGpsReading g;
    if (gps.read(g)) {
      withMutex([&]() {
        shared.raw.gps.lat       = g.lat;
        shared.raw.gps.lon       = g.lon;
        shared.raw.gps.fix       = g.fix;
        shared.raw.gps.sats      = g.sats;
        shared.raw.gps.speedMps  = g.speedMps;
        shared.raw.gps.lastFixMs = g.lastFixMs;
      });
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
  const TickType_t xFrequency = pdMS_TO_TICKS(PHYSICS_LOOP_MS);
  TickType_t lastWakeTime     = xTaskGetTickCount();

  for (;;) {
    unsigned long now = micros();
    float dt          = (now - lastGyroMicros) / 1000000.0f;
    lastGyroMicros    = now;

#if defined(SIM)
    // On-chip sim: advance the QuadSim physics with the last commanded mix and
    // write the resulting attitude/altitude/GPS back as fake sensor readings.
    // This runs BEFORE the phase's physicsTick, so the controller sees fresh state.
    onboardSimStep(dt);
#else
    RawImuReading r;
    imu.read(r, dt);
    float baroAlt = altimeter.readAltitudeFt();

    withMutex([&]() {
      shared.raw.imu.gyroX      = r.gyroX;
      shared.raw.imu.gyroY      = r.gyroY;
      shared.raw.imu.gyroZ      = r.gyroZ;
      shared.raw.imu.accX       = r.accX;
      shared.raw.imu.accY       = r.accY;
      shared.raw.imu.accZ       = r.accZ;
      shared.raw.imu.temp       = r.temp;
      shared.raw.baroAltitudeFt = baroAlt;
    });
#endif

    FlightPhase phase;
    withMutex([&]() { phase = shared.phase; });

    phaseFor(phase)->physicsTick(dt);
    vTaskDelayUntil(&lastWakeTime, xFrequency);
  }
}

// Each sensor/actuator brings itself up via its own service's begin() (called
// in setup below). No init helpers live here anymore.

// ==========================================
// SETUP
// ==========================================
void setup() {
  sharedDataMutex = xSemaphoreCreateMutex(); 
  serialMutex     = xSemaphoreCreateMutex();
  
  Serial.begin(115200);
#ifdef SIM
  // ESP32-S3 native USB CDC takes a moment to enumerate after reset.
  // Without this delay, early Serial output is dropped before the host
  // sees the port. 2s is enough for macOS to reconnect and open the port.
  delay(2000);
#endif
#ifndef SIM
  Wire.begin(16, 15);
  imu.begin();
  compass.begin();
  logLine("[COMPASS] QMC5883L ready.");
  gps.begin();
  motors.begin();
  logLine("[ESC] DShot600 channels initialized, all motors disarmed.");
  altimeter.begin();   // samples + locks in the ground-altitude reference
#else
  // In sim there is no sensor hardware: the QuadSim physics (OnboardSim) provides
  // every sensor reading. Bring it up: mount LittleFS, open the flight log, seed
  // GPS at home.
  logLine("[SENSORS] Sim mode -- sensors driven by on-chip QuadSim physics.");
  onboardSimBegin();
#endif

  xTaskCreatePinnedToCore(navigationTask, "NavTask",    8192, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(physicsTask,    "PhysicsTask", 8192, NULL, 2, NULL, 1);
#ifndef SIM
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
#ifdef SIM
  // In sim, the flight is autonomous; this only listens for the DUMPLOG command
  // to stream the recorded flight log back over USB.
  parseSimInput();
#else
  // On real hardware everything runs in the nav/physics/CRSF tasks;
  // the Arduino loop has nothing to do.
  vTaskDelay(pdMS_TO_TICKS(100));
#endif
}
