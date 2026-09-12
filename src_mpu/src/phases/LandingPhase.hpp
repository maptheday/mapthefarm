#pragma once

// ============================================================================
// LANDING -- walk the target altitude down at LAND_DESCENT_RATE_FPS until it
// reaches the ground, then go to LANDED (which cuts the motors). Also the
// direct abort target when GPS is lost and RTL isn't possible.
// ============================================================================

#include "../IFlightPhase.hpp"
#include "../PhaseState.hpp"
#include "../FlightRuntime.hpp"

class LandingPhase : public IFlightPhase {
public:
  FlightPhase id() const override { return PHASE_LANDING; }

  void onEnter(const EnterContext& ctx) override {
    shared.cruise_landing.targetAltFt      = ctx.currentAltFt;
    shared.cruise_landing.yawTargetHeading = ctx.currentHeadingDeg;
    shared.cruise_landing.targetRollDeg    = 0.0f;
    shared.cruise_landing.targetPitchDeg   = 0.0f;
    shared.dashboard_landing.m1            = 0.0f;
    shared.dashboard_landing.m2            = 0.0f;
    shared.dashboard_landing.m3            = 0.0f;
    shared.dashboard_landing.m4            = 0.0f;
  }

  void navTick(float navDt) override {
    withMutex([&]() {
      shared.dashboard_landing.altitudeFt     = shared.raw.baroAltitudeFt;
      shared.dashboard_landing.compassHeading = shared.raw.compassHeadingDeg;
      shared.dashboard_landing.roll           = shared.raw.imu.gyroX;
      shared.dashboard_landing.pitch          = shared.raw.imu.gyroY;
      shared.dashboard_landing.yaw            = shared.raw.imu.gyroZ;
    });

    bool landed = false;
    withMutex([&]() {
      float newTarget = shared.cruise_landing.targetAltFt - (LAND_DESCENT_RATE_FPS * navDt);
      if (newTarget <= 0.0f) {
        shared.cruise_landing.targetAltFt = 0.0f;
        landed = true;
      } else {
        shared.cruise_landing.targetAltFt = newTarget;
      }
    });

    if (landed) {
      transitionTo(PHASE_LANDED, REASON_TOUCHDOWN);
      logLine("[NAV] Landed — motors disarmed.");
    }
  }

  void physicsTick(float dt) override {
    Cruise_Landing c;
    RawSensors     r;
    withMutex([&]() {
      c = shared.cruise_landing;
      r = shared.raw;
    });

    MotorMix mix = motorController.computeMotorMix(
      c.targetAltFt, c.targetRollDeg, c.targetPitchDeg, c.yawTargetHeading,
      r.baroAltitudeFt, r.imu.gyroX, r.imu.gyroY, r.compassHeadingDeg, r.imu.gyroZ, dt);

    writeMotorMix(mix);

    withMutex([&]() {
      shared.dashboard_landing.m1              = mix.m1;
      shared.dashboard_landing.m2              = mix.m2;
      shared.dashboard_landing.m3              = mix.m3;
      shared.dashboard_landing.m4              = mix.m4;
      shared.dashboard_landing.baseThrottle    = mix.baseThrottle;
      shared.dashboard_landing.rollCorrection  = mix.rollCorrection;
      shared.dashboard_landing.pitchCorrection = mix.pitchCorrection;
    });
  }

  void writeTelemetry(JsonDocument& doc) override {
    Cruise_Landing    c;
    Dashboard_Landing db;
    float heading;
    withMutex([&]() {
      c       = shared.cruise_landing;
      db      = shared.dashboard_landing;
      heading = shared.raw.compassHeadingDeg;
    });

    doc["targetFt"]       = c.targetAltFt;
    doc["m1"]             = (int)(db.m1 * 100);
    doc["m2"]             = (int)(db.m2 * 100);
    doc["m3"]             = (int)(db.m3 * 100);
    doc["m4"]             = (int)(db.m4 * 100);
    doc["compassHeading"] = heading;
    doc["navActive"]      = false;
  }
};
