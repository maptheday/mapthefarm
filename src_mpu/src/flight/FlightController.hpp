#pragma once

// ============================================================================
// THE FLIGHT CONTROLLER -- public API.
//
// Every app (src/apps/*) includes this file, exactly once, from its main.cpp.
// The flight controller itself has no built-in settings, no test modes, and no
// idea which app is running it. An app:
//
//   1. loads the settings          fc::loadSettings(errors, {overrides...})
//   2. picks what to plug in       the seven plugs in FlightIo.hpp: the real
//                                  drivers (HardwareIo) or fakes (the sim app)
//   3. starts it                   fc::begin(io)
//   4. (optionally) drives it      fc::start(), fc::land(), ... -- the same
//                                  commands the radio switches send
//
// Under the hood: three FreeRTOS loops on the two cores (see "Idea 1" in
// CLAUDE.md): a 10 Hz navigation loop and a 200 Hz physics loop, each asking
// the current phase what to do, and a radio loop that turns radio frames into
// switch flips and stick positions.
// ============================================================================

#include <Arduino.h>

#include "state/FlightSettings.hpp"
#include "models/FlightModel.hpp"
#include "models/SensorTypes.hpp"
#include "models/ControlTypes.hpp"
#include "state/PhaseState.hpp"
#include "FlightIo.hpp"
#include "services/Log.hpp"
#include "services/MotorController.hpp"
#include "services/Motors.hpp"
#include "services/Battery.hpp"
#include "services/Mission.hpp"
#include "phases/IFlightPhase.hpp"
#include "phases/PhaseRegistry.hpp"
#include "phases/PhaseMachine.hpp"
#include "services/Failsafes.hpp"
#include "services/RcInput.hpp"

// ---------------------------------------------------------------------------
// The one instance of each service and of the shared notebook. Every service
// header declares these `extern`; they're born here, once. (That's why an app
// includes this file exactly once.)
// ---------------------------------------------------------------------------
SemaphoreHandle_t    sharedDataMutex;
SemaphoreHandle_t    serialMutex;
volatile SharedState shared;

Motors          motors;           // the 4 motors (through the motors plug)
Battery         battery;          // fuel gauge: voltage + estimated mAh
MotorController motorController;  // PID + motor mixing

namespace fc {

namespace detail {
// The two locks ("pens"), created on first use so an app can log before begin().
inline void ensureMutexes() {
  if (!sharedDataMutex) sharedDataMutex = xSemaphoreCreateMutex();
  if (!serialMutex)     serialMutex     = xSemaphoreCreateMutex();
}

// 10 Hz: fresh GPS + compass into the notebook, then the phase's navTick.
inline void navigationTask(void*) {
  const TickType_t period = pdMS_TO_TICKS(NAV_LOOP_MS);
  TickType_t lastWake     = xTaskGetTickCount();
  const float navDt       = NAV_LOOP_MS / 1000.0f;

  for (;;) {
    RawGpsReading gps     = flightIo().gps->read();
    float         heading = flightIo().compass->readHeadingDeg();
    withMutex([&]() {
      if (gps.fix) {
        shared.raw.gps.lat       = gps.lat;
        shared.raw.gps.lon       = gps.lon;
        shared.raw.gps.fix       = true;
        shared.raw.gps.sats      = gps.sats;
        shared.raw.gps.speedMps  = gps.speedMps;
        shared.raw.gps.lastFixMs = gps.lastFixMs;
      } else {
        // Keep the last good position and its time: the GPS-loss failsafe
        // measures how long it's been since then.
        shared.raw.gps.fix = false;
      }
      shared.raw.compassHeadingDeg = heading;
    });

    FlightPhase phase;
    withMutex([&]() { phase = shared.phase; });
    phaseFor(phase)->navTick(navDt);
    vTaskDelayUntil(&lastWake, period);
  }
}

// 200 Hz: fresh attitude + altitude, the phase's physicsTick (which drives the
// motors), then the battery fuel gauge.
inline void physicsTask(void*) {
  const TickType_t period = pdMS_TO_TICKS(PHYSICS_LOOP_MS);
  TickType_t lastWake     = xTaskGetTickCount();
  unsigned long lastMicros = micros();

  for (;;) {
    unsigned long now = micros();
    float dt          = (now - lastMicros) / 1000000.0f;
    lastMicros        = now;

    RawImuReading imu       = flightIo().imu->read(dt);
    float         altFt     = flightIo().altimeter->readFt();
    float         packVolts = flightIo().battery->readPackVolts();
    withMutex([&]() {
      shared.raw.imu.gyroX      = imu.gyroX;   // fused roll  (naming quirk)
      shared.raw.imu.gyroY      = imu.gyroY;   // fused pitch
      shared.raw.imu.gyroZ      = imu.gyroZ;   // fused yaw
      shared.raw.imu.yawRateDps = imu.yawRateDps;
      shared.raw.imu.accX       = imu.accX;
      shared.raw.imu.accY       = imu.accY;
      shared.raw.imu.accZ       = imu.accZ;
      shared.raw.imu.temp       = imu.temp;
      shared.raw.baroAltitudeFt = altFt;
    });

    FlightPhase phase;
    withMutex([&]() { phase = shared.phase; });
    phaseFor(phase)->physicsTick(dt);

    // Fuel gauge: filtered voltage, estimated current and mAh used, and the
    // OK / WARNING / CRITICAL state the battery failsafe acts on.
    battery.update(packVolts, motors.lastMix, dt);
    RawBattery b = battery.reading();
    withMutex([&]() {
      shared.raw.battery.present   = b.present;
      shared.raw.battery.packVolts = b.packVolts;
      shared.raw.battery.cellVolts = b.cellVolts;
      shared.raw.battery.amps      = b.amps;
      shared.raw.battery.mAhUsed   = b.mAhUsed;
      shared.raw.battery.state     = b.state;
    });
    vTaskDelayUntil(&lastWake, period);
  }
}

// Every 2 ms: any new radio frame -> sticks, switch flips, "radio is alive".
// (A CRSF frame arrives every ~4 ms, so this never falls behind.)
inline void radioTask(void*) {
  for (;;) {
    RadioFrame frame;
    if (flightIo().radio->read(frame)) handleRadioFrame(frame);
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}
}  // namespace detail

// ===========================================================================
// 1. Settings
// ===========================================================================

// Load /flightsettings.json into the flight controller, then any override
// files on top, in order (like appsettings.Development.json in .NET):
//     fc::loadSettings(errors)                       base file only
//     fc::loadSettings(errors, {"first_mission"})    + /flightsettings/first_mission.json
// Returns false, with every problem listed in `errors`, if a file is missing
// or anything is incomplete. Every override asked for is required.
inline bool loadSettings(String& errors, const std::vector<String>& overrides = {}) {
  detail::ensureMutexes();
  bool ok = loadFlightSettings(flightSettingsStorage(), errors, overrides);
  if (ok) {
    String files = "flightsettings.json";
    for (const String& name : overrides) files += " + flightsettings/" + name + ".json";
    logLine("[FC] Settings loaded: " + files);
  }
  return ok;
}

// Is there an override file with this name on the drone? (For apps that pick
// up an override only when one exists, like the sim's per-scenario files.)
inline bool hasSettingsOverride(const String& name) {
  return LittleFS.begin(false) && LittleFS.exists(settingsOverridePath(name).c_str());
}

// Print a message and stop forever. For "can't safely continue" at startup.
inline void halt(const String& why) {
  Serial.println("[HALT] " + why);
  Serial.println("[HALT] The drone will not fly. Fix the above and restart.");
  while (true) delay(1000);
}

// ===========================================================================
// 2. Start
// ===========================================================================

// Plug in the sensors, motors and radio, bring each one up, and start the
// flight loops. Every plug is required except the radio: with no radio the
// drone has no pilot, so nothing ever tells it to take off. The drone starts
// in PARKED (motors off) and flies the mission route from the settings unless
// the app calls setMission() first. The app must call Serial.begin() first.
inline void begin(const FlightIo& io) {
  detail::ensureMutexes();
  if (!io.imu || !io.altimeter || !io.gps || !io.compass || !io.battery || !io.motors)
    halt("fc::begin: a sensor or the motors aren't plugged in (see FlightIo.hpp).");

  motorController.configure(settings());
  missionRoute() = settings().mission.route;

  flightIo() = io;
  io.motors->begin();   // first, so the ESCs get "stopped" right away
  motors.disarmAll();
  io.imu->begin();
  io.compass->begin();
  io.gps->begin();
  io.battery->begin();
  io.altimeter->begin();   // samples + locks in "ground = 0 ft" -- keep the drone still
  if (io.radio) io.radio->begin();

  xTaskCreatePinnedToCore(detail::navigationTask, "NavTask",     8192, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(detail::physicsTask,    "PhysicsTask", 8192, NULL, 2, NULL, 1);
  if (io.radio) xTaskCreatePinnedToCore(detail::radioTask, "RadioTask", 4096, NULL, 1, NULL, 0);
  logLine("[FC] Flight controller running. Mission: " + settings().mission.name +
          " (" + String((int)missionRoute().size()) + " waypoints).");
}

// ===========================================================================
// 3. Commands -- the same things the radio switches do.
// ===========================================================================

inline void start()     { crsfHandleStart(); }     // take off / start the mission
inline void stop()      { crsfHandleStop(); }      // EMERGENCY: motors off now
inline void land()      { crsfHandleLand(); }      // land gently where it is
inline void manualOn()  { crsfHandleManualOn(); }  // hand control to the sticks
inline void manualOff() { crsfHandleManualOff(); } // back to auto-hover

// Replace the route MISSION flies. Only on the ground (PARKED or LANDED).
inline bool setMission(const std::vector<Waypoint>& route) {
  FlightPhase phase;
  withMutex([&]() { phase = shared.phase; });
  if (route.empty() || (phase != PHASE_PARKED && phase != PHASE_LANDED)) {
    logLine("[FC] setMission refused (empty route, or not on the ground).");
    return false;
  }
  missionRoute() = route;
  logLine("[FC] Mission set: " + String((int)route.size()) + " waypoints.");
  return true;
}

// Ground maintenance: calibrate the compass (motors stay off). Only when parked.
inline void calibrateCompass() {
  FlightPhase phase;
  withMutex([&]() { phase = shared.phase; });
  if (phase == PHASE_PARKED) transitionTo(PHASE_CALIBRATE);
  else logLine("[FC] calibrateCompass refused (not parked).");
}

// ===========================================================================
// 4. Read-only state -- for apps that watch the flight (logging, the sim).
// ===========================================================================

inline FlightPhase phase()        { FlightPhase p; withMutex([&]() { p = shared.phase; }); return p; }
inline MotorMix    lastMotorMix() { return motors.lastMix; }
inline RawBattery  batteryState() { RawBattery b; withMutex([&]() { b = shared.raw.battery; }); return b; }
inline RawSensors  sensors()      { RawSensors r; withMutex([&]() { r = shared.raw; }); return r; }
inline int         missionWaypointIndex() { int i; withMutex([&]() { i = shared.trip_mission.currentWP; }); return i; }
inline void        launchPoint(double& lat, double& lon) {
  withMutex([&]() { lat = shared.trip_mission.launchLat; lon = shared.trip_mission.launchLon; });
}

}  // namespace fc
