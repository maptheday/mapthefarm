#pragma once

// ============================================================================
// LANDING -- walk the target altitude down at LAND_DESCENT_RATE_FPS until it
// reaches the ground, then go to LANDED (which cuts the motors). Also the
// direct abort target when GPS is lost and RTL isn't possible.
// ============================================================================

#include "IFlightPhase.hpp"
#include "../state/PhaseState.hpp"
#include "../state/FlightConfig.hpp"        // LAND_DESCENT_RATE_FPS
#include "../services/Motors.hpp"           // motors
#include "../services/MotorController.hpp"  // motorController
#include "../services/NavMath.hpp"          // gpsDistanceMeters, gpsBearing, bearingToNorthEast
#include "../services/Log.hpp"              // logLine
#include "PhaseSwitch.hpp"                  // transitionTo

class LandingPhase : public IFlightPhase {
public:
  FlightPhase id() const override { return PHASE_LANDING; }

  void onEnter(const EnterContext& ctx) override {
    shared.cruise_landing.targetAltFt      = ctx.currentAltFt;
    shared.cruise_landing.yawTargetHeading = ctx.currentHeadingDeg;
    shared.cruise_landing.targetRollDeg    = 0.0f;
    shared.cruise_landing.targetPitchDeg   = 0.0f;
    shared.trip_landing.anchorLat          = ctx.currentLat;  // hold here while descending
    shared.trip_landing.anchorLon          = ctx.currentLon;
    shared.dashboard_landing.m1            = 0.0f;
    shared.dashboard_landing.m2            = 0.0f;
    shared.dashboard_landing.m3            = 0.0f;
    shared.dashboard_landing.m4            = 0.0f;
  }

  void navTick(float navDt) override {
    RawGpsReading gps;
    Trip_Landing  trip;
    withMutex([&]() {
      gps  = shared.raw.gps;
      trip = shared.trip_landing;
      shared.dashboard_landing.altitudeFt     = shared.raw.baroAltitudeFt;
      shared.dashboard_landing.compassHeading = shared.raw.compassHeadingDeg;
      shared.dashboard_landing.roll           = shared.raw.imu.gyroX;
      shared.dashboard_landing.pitch          = shared.raw.imu.gyroY;
      shared.dashboard_landing.yaw            = shared.raw.imu.gyroZ;
    });

    // Hold over the touchdown spot while descending so we don't coast off it --
    // but ONLY with a GPS fix. Without one (the GPS-loss abort case) we can't
    // hold position, so we descend level, exactly as before.
    if (gps.fix) {
      float distM   = gpsDistanceMeters(gps.lat, gps.lon, trip.anchorLat, trip.anchorLon);
      float bearing = gpsBearing(gps.lat, gps.lon, trip.anchorLat, trip.anchorLon);
      float northM;
      float eastM;
      bearingToNorthEast(distM, bearing, northM, eastM);
      withMutex([&]() {
        shared.cruise_landing.targetRollDeg  = motorController.eastNavigationCorrection(eastM, navDt);
        shared.cruise_landing.targetPitchDeg = motorController.northNavigationCorrection(northM, navDt);
      });
    } else {
      withMutex([&]() {
        shared.cruise_landing.targetRollDeg  = 0.0f;
        shared.cruise_landing.targetPitchDeg = 0.0f;
      });
    }

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

    motors.writeMix(mix);

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
};
