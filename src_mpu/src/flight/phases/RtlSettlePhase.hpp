#pragma once

// ============================================================================
// RTL_SETTLE -- final step of the comeback.
// Hover in place over the launch point for settings().safety.rtlSettleMs to bleed off any
// leftover momentum, then begin LANDING.
// ============================================================================

#include "IFlightPhase.hpp"
#include "../state/PhaseState.hpp"
#include "../state/FlightSettings.hpp"        // settings().safety.rtlAltitudeFt, settings().safety.rtlSettleMs
#include "../services/Motors.hpp"           // Motors
#include "../services/MotorController.hpp"  // MotorController
#include "../services/NavMath.hpp"          // gpsDistanceMeters, gpsBearing, bearingToNorthEast
#include "../services/Log.hpp"              // logLine
#include "../services/Failsafes.hpp"        // Failsafes
#include "PhaseSwitch.hpp"                  // transitionTo

class RtlSettlePhase : public IFlightPhase {
public:
  RtlSettlePhase(Motors* motors, MotorController* motorController, Failsafes* failsafes)
    : motors_(motors), motorController_(motorController), failsafes_(failsafes) {}

  FlightPhase id() const override { return PHASE_RTL_SETTLE; }

  void onEnter(const EnterContext& ctx) override {
    shared.trip_rtlSettle.settleStartMs      = ctx.now;
    shared.trip_rtlSettle.launchLat          = ctx.carriedLaunchLat;
    shared.trip_rtlSettle.launchLon          = ctx.carriedLaunchLon;
    shared.cruise_rtlSettle.targetAltFt      = settings().safety.rtlAltitudeFt;
    shared.cruise_rtlSettle.yawTargetHeading = ctx.currentHeadingDeg;
    shared.cruise_rtlSettle.targetRollDeg    = 0.0f;
    shared.cruise_rtlSettle.targetPitchDeg   = 0.0f;
    shared.dashboard_rtlSettle.m1            = 0.0f;
    shared.dashboard_rtlSettle.m2            = 0.0f;
    shared.dashboard_rtlSettle.m3            = 0.0f;
    shared.dashboard_rtlSettle.m4            = 0.0f;
  }

  void navTick(float navDt) override {
    unsigned long settleStart;
    RawGpsReading gps;
    Trip_RtlSettle trip;
    withMutex([&]() {
      settleStart = shared.trip_rtlSettle.settleStartMs;
      gps         = shared.raw.gps;
      trip        = shared.trip_rtlSettle;

      shared.dashboard_rtlSettle.altitudeFt     = shared.raw.baroAltitudeFt;
      shared.dashboard_rtlSettle.compassHeading = shared.raw.compassHeadingDeg;
      shared.dashboard_rtlSettle.roll           = shared.raw.imu.gyroX;
      shared.dashboard_rtlSettle.pitch          = shared.raw.imu.gyroY;
      shared.dashboard_rtlSettle.yaw            = shared.raw.imu.gyroZ;
    });
    // Battery low on the way home (or while settling)? Land right here.
    if (failsafes_->checkBattery()) return;

    // Actively hold over the launch pad while settling: lean back toward it to
    // brake off any leftover momentum, instead of going level and coasting away
    // (same nav math MISSION/RTL_RETURN use).
    float distM   = gpsDistanceMeters(gps.lat, gps.lon, trip.launchLat, trip.launchLon);
    float bearing = gpsBearing(gps.lat, gps.lon, trip.launchLat, trip.launchLon);
    float northM;
    float eastM;
    bearingToNorthEast(distM, bearing, northM, eastM);
    withMutex([&]() {
      float forwardM, rightM;   // world north/east -> the drone's own forward/right
      northEastToForwardRight(northM, eastM, shared.raw.compassHeadingDeg, forwardM, rightM);
      shared.cruise_rtlSettle.targetRollDeg  = motorController_->rightNavigationCorrection(rightM, navDt);
      shared.cruise_rtlSettle.targetPitchDeg = motorController_->forwardNavigationCorrection(forwardM, navDt);
    });

    if (millis() - settleStart >= settings().safety.rtlSettleMs) {
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

    MotorMix mix = motorController_->computeMotorMix(
      c.targetAltFt, c.targetRollDeg, c.targetPitchDeg, c.yawTargetHeading,
      r.baroAltitudeFt, r.imu.gyroX, r.imu.gyroY, r.compassHeadingDeg, r.imu.yawRateDps, dt);

    motors_->writeMix(mix);

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

private:
  Motors*           motors_;
  MotorController*  motorController_;
  Failsafes*        failsafes_;
};
