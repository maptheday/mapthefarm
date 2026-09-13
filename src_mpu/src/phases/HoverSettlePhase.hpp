#pragma once

// ============================================================================
// HOVER_SETTLE -- after the mission's last waypoint, hover in place for
// MISSION_COMPLETE_HOVER_MS to bleed off momentum, then begin LANDING.
// ============================================================================

#include "IFlightPhase.hpp"
#include "../state/PhaseState.hpp"
#include "../state/FlightConfig.hpp"        // MISSION_COMPLETE_HOVER_MS
#include "../services/Motors.hpp"           // motors
#include "../services/MotorController.hpp"  // motorController
#include "../services/Log.hpp"              // logLine
#include "PhaseSwitch.hpp"                  // transitionTo

class HoverSettlePhase : public IFlightPhase {
public:
  FlightPhase id() const override { return PHASE_HOVER_SETTLE; }

  void onEnter(const EnterContext& ctx) override {
    shared.trip_hoverSettle.enteredAtMs        = ctx.now;
    shared.cruise_hoverSettle.targetAltFt      = ctx.currentAltFt;
    shared.cruise_hoverSettle.yawTargetHeading = ctx.currentHeadingDeg;
    shared.cruise_hoverSettle.targetRollDeg    = 0.0f;
    shared.cruise_hoverSettle.targetPitchDeg   = 0.0f;
    shared.dashboard_hoverSettle.m1            = 0.0f;
    shared.dashboard_hoverSettle.m2            = 0.0f;
    shared.dashboard_hoverSettle.m3            = 0.0f;
    shared.dashboard_hoverSettle.m4            = 0.0f;
  }

  void navTick(float /*navDt*/) override {
    withMutex([&]() {
      shared.dashboard_hoverSettle.altitudeFt     = shared.raw.baroAltitudeFt;
      shared.dashboard_hoverSettle.compassHeading = shared.raw.compassHeadingDeg;
      shared.dashboard_hoverSettle.roll           = shared.raw.imu.gyroX;
      shared.dashboard_hoverSettle.pitch          = shared.raw.imu.gyroY;
      shared.dashboard_hoverSettle.yaw            = shared.raw.imu.gyroZ;
    });

    unsigned long enteredAt;
    withMutex([&]() { enteredAt = shared.trip_hoverSettle.enteredAtMs; });

    if (millis() - enteredAt >= MISSION_COMPLETE_HOVER_MS) {
      transitionTo(PHASE_LANDING, REASON_HOVER_COMPLETE);
      logLine("[NAV] Hover complete — beginning automatic landing.");
    }
  }

  void physicsTick(float dt) override {
    Cruise_HoverSettle c;
    RawSensors         r;
    withMutex([&]() {
      c = shared.cruise_hoverSettle;
      r = shared.raw;
    });

    MotorMix mix = motorController.computeMotorMix(
      c.targetAltFt, c.targetRollDeg, c.targetPitchDeg, c.yawTargetHeading,
      r.baroAltitudeFt, r.imu.gyroX, r.imu.gyroY, r.compassHeadingDeg, r.imu.gyroZ, dt);

    motors.writeMix(mix);

    withMutex([&]() {
      shared.dashboard_hoverSettle.m1              = mix.m1;
      shared.dashboard_hoverSettle.m2              = mix.m2;
      shared.dashboard_hoverSettle.m3              = mix.m3;
      shared.dashboard_hoverSettle.m4              = mix.m4;
      shared.dashboard_hoverSettle.baseThrottle    = mix.baseThrottle;
      shared.dashboard_hoverSettle.rollCorrection  = mix.rollCorrection;
      shared.dashboard_hoverSettle.pitchCorrection = mix.pitchCorrection;
    });
  }
};
