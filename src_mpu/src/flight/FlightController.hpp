#pragma once

// ============================================================================
// THE FLIGHT CONTROLLER -- public API.
//
// Every app (src/apps/*) includes this file, exactly once, from its main.cpp.
// The flight controller itself has no built-in settings, no test modes, and no
// idea which app is running it. An app:
//
//   1. loads the settings          fc::loadSettings(errors)   (flightsettings.json)
//   2. (optionally) adjusts them   fc::settingsForEdit()      before begin() only
//   3. picks where inputs come     HardwareIo (real drone) or its own FlightIo
//      from / motor commands go
//   4. starts it                   fc::begin(io)
//   5. drives it                   fc::start(), fc::land(), fc::setMission(), ...
//                                  the same commands the radio switches send
//
// Under the hood: two FreeRTOS loops on the two cores (see "Idea 1" in
// CLAUDE.md): a 10 Hz navigation loop and a 200 Hz physics loop, each asking
// the current phase what to do.
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
#include "services/Compass.hpp"
#include "services/Gps.hpp"
#include "services/Imu.hpp"
#include "services/Altimeter.hpp"
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

Motors          motors;           // the 4 motors (through the FlightIo)
Battery         battery;          // fuel gauge: voltage + estimated mAh
MotorController motorController;  // PID + motor mixing
Compass         compass;          // magnetometer + calibration (real hardware)
Gps             gps;              // GPS receiver (real hardware)
Imu             imu;              // accel/gyro + attitude filter (real hardware)
Altimeter       altimeter;        // barometer height-above-ground (real hardware)

namespace fc {

namespace detail {
inline FlightIo*& io() { static FlightIo* p = nullptr; return p; }

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
    SlowInputs in;
    io()->readSlow(in);
    withMutex([&]() {
      if (in.gps.fix) {
        shared.raw.gps.lat       = in.gps.lat;
        shared.raw.gps.lon       = in.gps.lon;
        shared.raw.gps.fix       = true;
        shared.raw.gps.sats      = in.gps.sats;
        shared.raw.gps.speedMps  = in.gps.speedMps;
        shared.raw.gps.lastFixMs = in.gps.lastFixMs;
      } else {
        // Keep the last good position and its time: the GPS-loss failsafe
        // measures how long it's been since then.
        shared.raw.gps.fix = false;
      }
      shared.raw.compassHeadingDeg = in.compassHeadingDeg;
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

    FastInputs in;
    io()->readFast(dt, in);
    withMutex([&]() {
      shared.raw.imu.gyroX      = in.imu.gyroX;   // fused roll  (naming quirk)
      shared.raw.imu.gyroY      = in.imu.gyroY;   // fused pitch
      shared.raw.imu.gyroZ      = in.imu.gyroZ;   // fused yaw
      shared.raw.imu.yawRateDps = in.imu.yawRateDps;
      shared.raw.imu.accX       = in.imu.accX;
      shared.raw.imu.accY       = in.imu.accY;
      shared.raw.imu.accZ       = in.imu.accZ;
      shared.raw.imu.temp       = in.imu.temp;
      shared.raw.baroAltitudeFt = in.baroAltitudeFt;
    });

    FlightPhase phase;
    withMutex([&]() { phase = shared.phase; });
    phaseFor(phase)->physicsTick(dt);

    // Fuel gauge: filtered voltage, estimated current and mAh used, and the
    // OK / WARNING / CRITICAL state the battery failsafe acts on.
    battery.update(in.packVolts, motors.lastMix, dt);
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
}  // namespace detail

// ===========================================================================
// 1. Settings
// ===========================================================================

// Load /flightsettings.json into the flight controller. Returns false, with
// every problem listed in `errors`, if the file is missing or incomplete.
inline bool loadSettings(String& errors) {
  detail::ensureMutexes();
  return loadFlightSettings(flightSettingsStorage(), errors);
}

// Change a setting from code (e.g. a sim scenario shrinking the geofence).
// Only before fc::begin(): once flying, the settings are fixed.
inline FlightSettings& settingsForEdit() { return flightSettingsStorage(); }

// Print a message and stop forever. For "can't safely continue" at startup.
inline void halt(const String& why) {
  Serial.println("[HALT] " + why);
  Serial.println("[HALT] The drone will not fly. Fix the above and restart.");
  while (true) delay(1000);
}

// ===========================================================================
// 2. Start
// ===========================================================================

// Bring up the IO and start both flight loops. The drone starts in PARKED
// (motors off) and flies the mission route from the settings unless the app
// calls setMission() first. The app must call Serial.begin() first.
inline void begin(FlightIo& io) {
  detail::ensureMutexes();

  motorController.configure(settings());
  missionRoute() = settings().mission.route;

  detail::io() = &io;
  motors.attach(io);
  io.begin();
  motors.disarmAll();

  xTaskCreatePinnedToCore(detail::navigationTask, "NavTask",     8192, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(detail::physicsTask,    "PhysicsTask", 8192, NULL, 2, NULL, 1);
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

// Stick positions for MANUAL mode (what the radio would send).
// throttle 0..1 (0.5 = hold height); roll/pitch/yaw -1..1 (0 = centered).
inline void setSticks(float throttle, float roll, float pitch, float yaw) {
  withMutex([&]() {
    shared.sticks.throttle = throttle;
    shared.sticks.roll     = roll;
    shared.sticks.pitch    = pitch;
    shared.sticks.yaw      = yaw;
  });
}

// Tell the flight controller a radio is alive right now (arms the radio-loss
// failsafe; see Failsafes.hpp). The real radio task does this on every frame.
inline void radioHeartbeat() {
  withMutex([&]() { shared.rcLastFrameMs = millis(); });
}

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
