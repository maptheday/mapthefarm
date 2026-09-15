#pragma once

// ============================================================================
// RTL_CLIMB -- first step of the "uh-oh, come home" comeback.
// Rise straight up to RTL_ALTITUDE_FT (a safe height to fly home at), holding
// the current heading and staying level, then hand off to RTL_RETURN.
// Entered automatically by a failsafe (max flight time / geofence).
// ============================================================================

#include "IFlightPhase.hpp"
#include "../state/PhaseState.hpp"
#include "../state/FlightConfig.hpp"        // RTL_ALTITUDE_FT
#include "../services/Motors.hpp"           // motors
#include "../services/MotorController.hpp"  // motorController
#include "../services/Log.hpp"              // logLine
#include "PhaseSwitch.hpp"                  // transitionTo

class RtlClimbPhase : public IFlightPhase {
public:
  FlightPhase id() const override { return PHASE_RTL_CLIMB; }

  void onEnter(const EnterContext& ctx) override {
    shared.trip_rtlClimb.armedAtMs          = ctx.carriedArmedAtMs;
    shared.trip_rtlClimb.launchLat          = ctx.carriedLaunchLat;
    shared.trip_rtlClimb.launchLon          = ctx.carriedLaunchLon;
    shared.cruise_rtlClimb.targetAltFt      = RTL_ALTITUDE_FT;
    shared.cruise_rtlClimb.yawTargetHeading = ctx.currentHeadingDeg; // hold heading while climbing
    shared.cruise_rtlClimb.targetRollDeg    = 0.0f;
    shared.cruise_rtlClimb.targetPitchDeg   = 0.0f;
    shared.dashboard_rtlClimb.m1            = 0.0f;
    shared.dashboard_rtlClimb.m2            = 0.0f;
    shared.dashboard_rtlClimb.m3            = 0.0f;
    shared.dashboard_rtlClimb.m4            = 0.0f;
  }

  void navTick(float /*navDt*/) override {
    float currentAlt;
    withMutex([&]() {
      currentAlt = shared.raw.baroAltitudeFt;

      shared.dashboard_rtlClimb.altitudeFt     = shared.raw.baroAltitudeFt;
      shared.dashboard_rtlClimb.compassHeading = shared.raw.compassHeadingDeg;
      shared.dashboard_rtlClimb.roll           = shared.raw.imu.gyroX;
      shared.dashboard_rtlClimb.pitch          = shared.raw.imu.gyroY;
      shared.dashboard_rtlClimb.yaw            = shared.raw.imu.gyroZ;
    });

    if (currentAlt >= RTL_ALTITUDE_FT - 2.0f) {
      logLine("[RTL] Climb complete. Returning to launch.");
      transitionTo(PHASE_RTL_RETURN, REASON_RTL_CLIMB_COMPLETE);
    }
  }

  void physicsTick(float dt) override {
    Cruise_RtlClimb c;
    RawSensors      r;
    withMutex([&]() {
      c = shared.cruise_rtlClimb;
      r = shared.raw;
    });

    MotorMix mix = motorController.computeMotorMix(
      c.targetAltFt, c.targetRollDeg, c.targetPitchDeg, c.yawTargetHeading,
      r.baroAltitudeFt, r.imu.gyroX, r.imu.gyroY, r.compassHeadingDeg, r.imu.gyroZ, dt);

    motors.writeMix(mix);

    withMutex([&]() {
      shared.dashboard_rtlClimb.m1              = mix.m1;
      shared.dashboard_rtlClimb.m2              = mix.m2;
      shared.dashboard_rtlClimb.m3              = mix.m3;
      shared.dashboard_rtlClimb.m4              = mix.m4;
      shared.dashboard_rtlClimb.baseThrottle    = mix.baseThrottle;
      shared.dashboard_rtlClimb.rollCorrection  = mix.rollCorrection;
      shared.dashboard_rtlClimb.pitchCorrection = mix.pitchCorrection;
    });
  }
};
