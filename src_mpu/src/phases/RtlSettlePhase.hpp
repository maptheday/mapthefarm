#pragma once

// ============================================================================
// RTL_SETTLE -- final step of the comeback.
// Hover in place over the launch point for RTL_SETTLE_MS to bleed off any
// leftover momentum, then begin LANDING.
// ============================================================================

#include "IFlightPhase.hpp"
#include "../state/PhaseState.hpp"
#include "../state/FlightConfig.hpp"        // RTL_ALTITUDE_FT, RTL_SETTLE_MS
#include "../services/Motors.hpp"           // motors
#include "../services/MotorController.hpp"  // motorController
#include "../services/Log.hpp"              // logLine
#include "PhaseSwitch.hpp"                  // transitionTo

class RtlSettlePhase : public IFlightPhase {
public:
  FlightPhase id() const override { return PHASE_RTL_SETTLE; }

  void onEnter(const EnterContext& ctx) override {
    shared.trip_rtlSettle.settleStartMs      = ctx.now;
    shared.cruise_rtlSettle.targetAltFt      = RTL_ALTITUDE_FT;
    shared.cruise_rtlSettle.yawTargetHeading = ctx.currentHeadingDeg;
    shared.cruise_rtlSettle.targetRollDeg    = 0.0f;
    shared.cruise_rtlSettle.targetPitchDeg   = 0.0f;
    shared.dashboard_rtlSettle.m1            = 0.0f;
    shared.dashboard_rtlSettle.m2            = 0.0f;
    shared.dashboard_rtlSettle.m3            = 0.0f;
    shared.dashboard_rtlSettle.m4            = 0.0f;
  }

  void navTick(float /*navDt*/) override {
    unsigned long settleStart;
    withMutex([&]() {
      settleStart = shared.trip_rtlSettle.settleStartMs;

      shared.dashboard_rtlSettle.altitudeFt     = shared.raw.baroAltitudeFt;
      shared.dashboard_rtlSettle.compassHeading = shared.raw.compassHeadingDeg;
      shared.dashboard_rtlSettle.roll           = shared.raw.imu.gyroX;
      shared.dashboard_rtlSettle.pitch          = shared.raw.imu.gyroY;
      shared.dashboard_rtlSettle.yaw            = shared.raw.imu.gyroZ;
    });

    if (millis() - settleStart >= RTL_SETTLE_MS) {
      logLine("[RTL] Settle complete. Beginning landing.");
      transitionTo(PHASE_LANDING, REASON_RTL_COMPLETE);
    }
  }

  void physicsTick(float dt) override {
    Cruise_RtlSettle c;
    RawSensors       r;
    withMutex([&]() {
      c = shared.cruise_rtlSettle;
      r = shared.raw;
    });

    MotorMix mix = motorController.computeMotorMix(
      c.targetAltFt, c.targetRollDeg, c.targetPitchDeg, c.yawTargetHeading,
      r.baroAltitudeFt, r.imu.gyroX, r.imu.gyroY, r.compassHeadingDeg, r.imu.gyroZ, dt);

    motors.writeMix(mix);

    withMutex([&]() {
      shared.dashboard_rtlSettle.m1              = mix.m1;
      shared.dashboard_rtlSettle.m2              = mix.m2;
      shared.dashboard_rtlSettle.m3              = mix.m3;
      shared.dashboard_rtlSettle.m4              = mix.m4;
      shared.dashboard_rtlSettle.baseThrottle    = mix.baseThrottle;
      shared.dashboard_rtlSettle.rollCorrection  = mix.rollCorrection;
      shared.dashboard_rtlSettle.pitchCorrection = mix.pitchCorrection;
    });
  }
};
