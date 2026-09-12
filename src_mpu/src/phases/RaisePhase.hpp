#pragma once

// ============================================================================
// RAISE (takeoff) -- ramp the target altitude from 0 up to TAKEOFF_ALTITUDE_FT
// at a fixed climb rate, then hand off to HOLD. Attitude is held level.
// ============================================================================

#include "../IFlightPhase.hpp"
#include "../PhaseState.hpp"
#include "../FlightRuntime.hpp"

class RaisePhase : public IFlightPhase {
public:
  FlightPhase id() const override { return PHASE_RAISE; }

  void onEnter(const EnterContext& ctx) override {
    // Reset all PID integrators so stale state from a previous flight cannot
    // corrupt the first takeoff ramp.
    motorController.reset();
    shared.trip_raise.armedAtMs          = ctx.now;
    shared.trip_raise.launchLat          = ctx.currentLat;
    shared.trip_raise.launchLon          = ctx.currentLon;
    shared.cruise_raise.targetAltFt      = 0.0f;  // start the ramp from 0
    shared.cruise_raise.yawTargetHeading = ctx.currentHeadingDeg;
    shared.cruise_raise.targetRollDeg    = 0.0f;
    shared.cruise_raise.targetPitchDeg   = 0.0f;
    shared.dashboard_raise.m1            = 0.0f;
    shared.dashboard_raise.m2            = 0.0f;
    shared.dashboard_raise.m3            = 0.0f;
    shared.dashboard_raise.m4            = 0.0f;
  }

  void navTick(float navDt) override {
    withMutex([&]() {
      shared.dashboard_raise.altitudeFt     = shared.raw.baroAltitudeFt;
      shared.dashboard_raise.compassHeading = shared.raw.compassHeadingDeg;
      shared.dashboard_raise.roll           = shared.raw.imu.gyroX;
      shared.dashboard_raise.pitch          = shared.raw.imu.gyroY;
      shared.dashboard_raise.yaw            = shared.raw.imu.gyroZ;
    });

    bool ready = false;
    withMutex([&]() {
      float newTarget = shared.cruise_raise.targetAltFt + (RAISE_CLIMB_RATE_FPS * navDt);
      if (newTarget >= TAKEOFF_ALTITUDE_FT) {
        shared.cruise_raise.targetAltFt = TAKEOFF_ALTITUDE_FT;
        ready = true;
      } else {
        shared.cruise_raise.targetAltFt = newTarget;
      }
    });

    if (ready) {
      logLine("[NAV] Takeoff altitude reached, transitioning to HOLD.");
      transitionTo(PHASE_HOLD, REASON_TAKEOFF_COMPLETE);
    }
  }

  void physicsTick(float dt) override {
    Cruise_Raise c;
    RawSensors   r;
    withMutex([&]() {
      c = shared.cruise_raise;
      r = shared.raw;
    });

    MotorMix mix = motorController.computeMotorMix(
      c.targetAltFt, c.targetRollDeg, c.targetPitchDeg, c.yawTargetHeading,
      r.baroAltitudeFt, r.imu.gyroX, r.imu.gyroY, r.compassHeadingDeg, r.imu.gyroZ, dt);

    writeMotorMix(mix);

    withMutex([&]() {
      shared.dashboard_raise.m1              = mix.m1;
      shared.dashboard_raise.m2              = mix.m2;
      shared.dashboard_raise.m3              = mix.m3;
      shared.dashboard_raise.m4              = mix.m4;
      shared.dashboard_raise.baseThrottle    = mix.baseThrottle;
      shared.dashboard_raise.rollCorrection  = mix.rollCorrection;
      shared.dashboard_raise.pitchCorrection = mix.pitchCorrection;
    });
  }

  void writeTelemetry(JsonDocument& doc) override {
    Cruise_Raise    c;
    Dashboard_Raise db;
    float heading;
    withMutex([&]() {
      c       = shared.cruise_raise;
      db      = shared.dashboard_raise;
      heading = shared.raw.compassHeadingDeg;
    });

    doc["targetFt"]        = c.targetAltFt;
    doc["m1"]              = (int)(db.m1 * 100);
    doc["m2"]              = (int)(db.m2 * 100);
    doc["m3"]              = (int)(db.m3 * 100);
    doc["m4"]              = (int)(db.m4 * 100);
    doc["baseThrottle"]    = (int)(db.baseThrottle * 100);
    doc["rollCorrection"]  = (int)(db.rollCorrection * 100);
    doc["pitchCorrection"] = (int)(db.pitchCorrection * 100);
    doc["compassHeading"]  = heading;
    doc["navActive"]       = false;
  }
};
