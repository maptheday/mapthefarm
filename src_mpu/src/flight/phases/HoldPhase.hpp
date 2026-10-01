#pragma once

// ============================================================================
// HOLD -- hover in place at the current altitude and wait. Reached after
// takeoff, or by aborting a mission. START from here begins the waypoint
// mission. Runs the core failsafes (max flight time / geofence / GPS loss).
// ============================================================================

#include "IFlightPhase.hpp"
#include "../state/PhaseState.hpp"
#include "../state/FlightSettings.hpp"        // settings().flight.takeoffAltitudeFt, settings().safety.maxFlightTimeMs
#include "../services/Motors.hpp"           // Motors
#include "../services/MotorController.hpp"  // MotorController
#include "../services/Failsafes.hpp"        // Failsafes

class HoldPhase : public IFlightPhase {
public:
  HoldPhase(Motors* motors, MotorController* motorController, Failsafes* failsafes)
    : motors_(motors), motorController_(motorController), failsafes_(failsafes) {}

  FlightPhase id() const override { return PHASE_HOLD; }

  void onEnter(const EnterContext& ctx) override {
    shared.trip_hold.armedAtMs          = ctx.carriedArmedAtMs;
    shared.trip_hold.launchLat          = ctx.carriedLaunchLat;
    shared.trip_hold.launchLon          = ctx.carriedLaunchLon;
    // Coming straight from takeoff, hold the planned takeoff altitude exactly;
    // otherwise hold wherever we are right now.
    shared.cruise_hold.targetAltFt      = (ctx.prevPhase == PHASE_RAISE)
                                            ? settings().flight.takeoffAltitudeFt : ctx.currentAltFt;
    shared.cruise_hold.yawTargetHeading = ctx.currentHeadingDeg;
    shared.cruise_hold.targetRollDeg    = 0.0f;
    shared.cruise_hold.targetPitchDeg   = 0.0f;
    shared.dashboard_hold.m1            = 0.0f;
    shared.dashboard_hold.m2            = 0.0f;
    shared.dashboard_hold.m3            = 0.0f;
    shared.dashboard_hold.m4            = 0.0f;
  }

  void navTick(float /*navDt*/) override {
    unsigned long armedAt;
    double launchLat;
    double launchLon;
    withMutex([&]() {
      shared.dashboard_hold.altitudeFt     = shared.raw.baroAltitudeFt;
      shared.dashboard_hold.compassHeading = shared.raw.compassHeadingDeg;
      shared.dashboard_hold.roll           = shared.raw.imu.gyroX;
      shared.dashboard_hold.pitch          = shared.raw.imu.gyroY;
      shared.dashboard_hold.yaw            = shared.raw.imu.gyroZ;
      armedAt                              = shared.trip_hold.armedAtMs;
      launchLat                            = shared.trip_hold.launchLat;
      launchLon                            = shared.trip_hold.launchLon;
    });

    failsafes_->checkCore(armedAt, launchLat, launchLon);
    if (failsafes_->checkRadio()) return;
    failsafes_->checkBattery();
  }

  void physicsTick(float dt) override {
    Cruise_Hold c;
    RawSensors  r;
    withMutex([&]() {
      c = shared.cruise_hold;
      r = shared.raw;
    });

    MotorMix mix = motorController_->computeMotorMix(
      c.targetAltFt, c.targetRollDeg, c.targetPitchDeg, c.yawTargetHeading,
      r.baroAltitudeFt, r.imu.gyroX, r.imu.gyroY, r.compassHeadingDeg, r.imu.yawRateDps, dt);

    motors_->writeMix(mix);

    withMutex([&]() {
      shared.dashboard_hold.m1              = mix.m1;
      shared.dashboard_hold.m2              = mix.m2;
      shared.dashboard_hold.m3              = mix.m3;
      shared.dashboard_hold.m4              = mix.m4;
      shared.dashboard_hold.baseThrottle    = mix.baseThrottle;
      shared.dashboard_hold.rollCorrection  = mix.rollCorrection;
      shared.dashboard_hold.pitchCorrection = mix.pitchCorrection;
    });
  }

private:
  Motors*           motors_;
  MotorController*  motorController_;
  Failsafes*        failsafes_;
};
