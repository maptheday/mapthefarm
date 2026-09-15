#pragma once

// ============================================================================
// RTL_RETURN -- second step of the comeback.
// Fly back over the launch point using the same GPS navigation as MISSION
// (turn to face home, tilt to move toward it), staying at RTL_ALTITUDE_FT.
// When within WAYPOINT_ACCEPT_RADIUS_M of launch, hand off to RTL_SETTLE.
// ============================================================================

#include "IFlightPhase.hpp"
#include "../state/PhaseState.hpp"
#include "../state/FlightConfig.hpp"        // RTL_ALTITUDE_FT, WAYPOINT_ACCEPT_RADIUS_M
#include "../services/Motors.hpp"           // motors
#include "../services/MotorController.hpp"  // motorController
#include "../services/NavMath.hpp"          // gpsDistanceMeters, gpsBearing, bearingToNorthEast
#include "../services/Log.hpp"              // logLine
#include "PhaseSwitch.hpp"                  // transitionTo

class RtlReturnPhase : public IFlightPhase {
public:
  FlightPhase id() const override { return PHASE_RTL_RETURN; }

  void onEnter(const EnterContext& ctx) override {
    shared.trip_rtlReturn.armedAtMs          = ctx.carriedArmedAtMs;
    shared.trip_rtlReturn.launchLat          = ctx.carriedLaunchLat;
    shared.trip_rtlReturn.launchLon          = ctx.carriedLaunchLon;
    shared.cruise_rtlReturn.targetAltFt      = RTL_ALTITUDE_FT;
    shared.cruise_rtlReturn.yawTargetHeading = ctx.currentHeadingDeg;
    shared.cruise_rtlReturn.targetRollDeg    = 0.0f;
    shared.cruise_rtlReturn.targetPitchDeg   = 0.0f;
    shared.dashboard_rtlReturn.m1            = 0.0f;
    shared.dashboard_rtlReturn.m2            = 0.0f;
    shared.dashboard_rtlReturn.m3            = 0.0f;
    shared.dashboard_rtlReturn.m4            = 0.0f;
  }

  void navTick(float navDt) override {
    RawGpsReading  gps;
    Trip_RtlReturn trip;
    withMutex([&]() {
      gps  = shared.raw.gps;
      trip = shared.trip_rtlReturn;

      shared.dashboard_rtlReturn.altitudeFt     = shared.raw.baroAltitudeFt;
      shared.dashboard_rtlReturn.compassHeading = shared.raw.compassHeadingDeg;
      shared.dashboard_rtlReturn.roll           = shared.raw.imu.gyroX;
      shared.dashboard_rtlReturn.pitch          = shared.raw.imu.gyroY;
      shared.dashboard_rtlReturn.yaw            = shared.raw.imu.gyroZ;
    });

    float distM   = gpsDistanceMeters(gps.lat, gps.lon, trip.launchLat, trip.launchLon);
    float bearing = gpsBearing(gps.lat, gps.lon, trip.launchLat, trip.launchLon);

    if (distM < WAYPOINT_ACCEPT_RADIUS_M) {
      logLine("[RTL] Arrived over launch pad. Settling.");
      transitionTo(PHASE_RTL_SETTLE, REASON_RTL_ARRIVED);
      return;
    }

    float northM;
    float eastM;
    bearingToNorthEast(distM, bearing, northM, eastM);

    withMutex([&]() {
      shared.cruise_rtlReturn.yawTargetHeading = bearing;
      shared.cruise_rtlReturn.targetRollDeg    = motorController.eastNavigationCorrection(eastM, navDt);
      shared.cruise_rtlReturn.targetPitchDeg   = motorController.northNavigationCorrection(northM, navDt);
    });
  }

  void physicsTick(float dt) override {
    Cruise_RtlReturn c;
    RawSensors       r;
    withMutex([&]() {
      c = shared.cruise_rtlReturn;
      r = shared.raw;
    });

    MotorMix mix = motorController.computeMotorMix(
      c.targetAltFt, c.targetRollDeg, c.targetPitchDeg, c.yawTargetHeading,
      r.baroAltitudeFt, r.imu.gyroX, r.imu.gyroY, r.compassHeadingDeg, r.imu.gyroZ, dt);

    motors.writeMix(mix);

    withMutex([&]() {
      shared.dashboard_rtlReturn.m1              = mix.m1;
      shared.dashboard_rtlReturn.m2              = mix.m2;
      shared.dashboard_rtlReturn.m3              = mix.m3;
      shared.dashboard_rtlReturn.m4              = mix.m4;
      shared.dashboard_rtlReturn.baseThrottle    = mix.baseThrottle;
      shared.dashboard_rtlReturn.rollCorrection  = mix.rollCorrection;
      shared.dashboard_rtlReturn.pitchCorrection = mix.pitchCorrection;
    });
  }
};
