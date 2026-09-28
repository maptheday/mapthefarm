#pragma once

// ============================================================================
// HOVER_SETTLE -- after the mission's last waypoint, hover in place for
// settings().flight.missionCompleteHoverMs to bleed off momentum, then begin LANDING.
// ============================================================================

#include "IFlightPhase.hpp"
#include "../state/PhaseState.hpp"
#include "../state/FlightSettings.hpp"        // settings().flight.missionCompleteHoverMs
#include "../services/Motors.hpp"           // motors
#include "../services/MotorController.hpp"  // motorController
#include "../services/NavMath.hpp"          // gpsDistanceMeters, gpsBearing, bearingToNorthEast
#include "../services/Log.hpp"              // logLine
#include "../services/Failsafes.hpp"        // checkBatteryFailsafe
#include "PhaseSwitch.hpp"                  // transitionTo

class HoverSettlePhase : public IFlightPhase {
public:
  FlightPhase id() const override { return PHASE_HOVER_SETTLE; }

  void onEnter(const EnterContext& ctx) override {
    shared.trip_hoverSettle.enteredAtMs        = ctx.now;
    shared.trip_hoverSettle.anchorLat          = ctx.currentLat;  // hold here, don't coast
    shared.trip_hoverSettle.anchorLon          = ctx.currentLon;
    shared.cruise_hoverSettle.targetAltFt      = ctx.currentAltFt;
    shared.cruise_hoverSettle.yawTargetHeading = ctx.currentHeadingDeg;
    shared.cruise_hoverSettle.targetRollDeg    = 0.0f;
    shared.cruise_hoverSettle.targetPitchDeg   = 0.0f;
    shared.dashboard_hoverSettle.m1            = 0.0f;
    shared.dashboard_hoverSettle.m2            = 0.0f;
    shared.dashboard_hoverSettle.m3            = 0.0f;
    shared.dashboard_hoverSettle.m4            = 0.0f;
  }

  void navTick(float navDt) override {
    RawGpsReading    gps;
    Trip_HoverSettle trip;
    withMutex([&]() {
      gps  = shared.raw.gps;
      trip = shared.trip_hoverSettle;
      shared.dashboard_hoverSettle.altitudeFt     = shared.raw.baroAltitudeFt;
      shared.dashboard_hoverSettle.compassHeading = shared.raw.compassHeadingDeg;
      shared.dashboard_hoverSettle.roll           = shared.raw.imu.gyroX;
      shared.dashboard_hoverSettle.pitch          = shared.raw.imu.gyroY;
      shared.dashboard_hoverSettle.yaw            = shared.raw.imu.gyroZ;
    });
    // Battery low on the way home (or while settling)? Land right here.
    if (checkBatteryFailsafe()) return;

    // Actively hold over the spot where the mission ended (lean back to brake
    // off leftover momentum) instead of coasting away while we settle.
    float distM   = gpsDistanceMeters(gps.lat, gps.lon, trip.anchorLat, trip.anchorLon);
    float bearing = gpsBearing(gps.lat, gps.lon, trip.anchorLat, trip.anchorLon);
    float northM;
    float eastM;
    bearingToNorthEast(distM, bearing, northM, eastM);
    withMutex([&]() {
      float forwardM, rightM;   // world north/east -> the drone's own forward/right
      northEastToForwardRight(northM, eastM, shared.raw.compassHeadingDeg, forwardM, rightM);
      shared.cruise_hoverSettle.targetRollDeg  = motorController.rightNavigationCorrection(rightM, navDt);
      shared.cruise_hoverSettle.targetPitchDeg = motorController.forwardNavigationCorrection(forwardM, navDt);
    });

    if (millis() - trip.enteredAtMs >= settings().flight.missionCompleteHoverMs) {
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
      r.baroAltitudeFt, r.imu.gyroX, r.imu.gyroY, r.compassHeadingDeg, r.imu.yawRateDps, dt);

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
