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
#include "../services/NavMath.hpp"          // gpsDistanceMeters, gpsBearing, bearingToNorthEast
#include "../services/Log.hpp"              // logLine
#include "PhaseSwitch.hpp"                  // transitionTo

class RtlClimbPhase : public IFlightPhase {
public:
  FlightPhase id() const override { return PHASE_RTL_CLIMB; }

  void onEnter(const EnterContext& ctx) override {
    shared.trip_rtlClimb.armedAtMs          = ctx.carriedArmedAtMs;
    shared.trip_rtlClimb.launchLat          = ctx.carriedLaunchLat;
    shared.trip_rtlClimb.launchLon          = ctx.carriedLaunchLon;
    shared.trip_rtlClimb.anchorLat          = ctx.currentLat;  // hold here while climbing
    shared.trip_rtlClimb.anchorLon          = ctx.currentLon;
    shared.cruise_rtlClimb.targetAltFt      = RTL_ALTITUDE_FT;
    shared.cruise_rtlClimb.yawTargetHeading = ctx.currentHeadingDeg; // hold heading while climbing
    shared.cruise_rtlClimb.targetRollDeg    = 0.0f;
    shared.cruise_rtlClimb.targetPitchDeg   = 0.0f;
    shared.dashboard_rtlClimb.m1            = 0.0f;
    shared.dashboard_rtlClimb.m2            = 0.0f;
    shared.dashboard_rtlClimb.m3            = 0.0f;
    shared.dashboard_rtlClimb.m4            = 0.0f;
  }

  void navTick(float navDt) override {
    float currentAlt;
    RawGpsReading gps;
    Trip_RtlClimb trip;
    withMutex([&]() {
      currentAlt = shared.raw.baroAltitudeFt;
      gps        = shared.raw.gps;
      trip       = shared.trip_rtlClimb;

      shared.dashboard_rtlClimb.altitudeFt     = shared.raw.baroAltitudeFt;
      shared.dashboard_rtlClimb.compassHeading = shared.raw.compassHeadingDeg;
      shared.dashboard_rtlClimb.roll           = shared.raw.imu.gyroX;
      shared.dashboard_rtlClimb.pitch          = shared.raw.imu.gyroY;
      shared.dashboard_rtlClimb.yaw            = shared.raw.imu.gyroZ;
    });

    // Climb IN PLACE: lean back toward the spot where RTL triggered to brake off
    // the mission's momentum, so we rise straight up instead of coasting away.
    float distM   = gpsDistanceMeters(gps.lat, gps.lon, trip.anchorLat, trip.anchorLon);
    float bearing = gpsBearing(gps.lat, gps.lon, trip.anchorLat, trip.anchorLon);
    float northM;
    float eastM;
    bearingToNorthEast(distM, bearing, northM, eastM);
    withMutex([&]() {
      shared.cruise_rtlClimb.targetRollDeg  = motorController.eastNavigationCorrection(eastM, navDt);
      shared.cruise_rtlClimb.targetPitchDeg = motorController.northNavigationCorrection(northM, navDt);
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
