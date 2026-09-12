#pragma once

// ============================================================================
// RTL (return to launch) -- the safety comeback. Three sub-steps:
//   CLIMB  -> rise to a safe altitude
//   RETURN -> fly back over the launch point
//   SETTLE -> pause a few seconds, then LAND
// Entered automatically by a failsafe (timeout / geofence) or the /rtl button.
// ============================================================================

#include "../IFlightPhase.hpp"
#include "../PhaseState.hpp"
#include "../FlightRuntime.hpp"

class RtlPhase : public IFlightPhase {
public:
  FlightPhase id() const override { return PHASE_RTL; }

  void onEnter(const EnterContext& ctx) override {
    shared.trip_rtl.armedAtMs          = ctx.carriedArmedAtMs;
    shared.trip_rtl.launchLat          = ctx.carriedLaunchLat;
    shared.trip_rtl.launchLon          = ctx.carriedLaunchLon;
    shared.trip_rtl.state              = RTL_CLIMB;
    shared.cruise_rtl.targetAltFt      = RTL_ALTITUDE_FT;
    shared.cruise_rtl.yawTargetHeading = ctx.currentHeadingDeg; // hold heading until climb done
    shared.cruise_rtl.targetRollDeg    = 0.0f;
    shared.cruise_rtl.targetPitchDeg   = 0.0f;
    shared.dashboard_rtl.m1            = 0.0f;
    shared.dashboard_rtl.m2            = 0.0f;
    shared.dashboard_rtl.m3            = 0.0f;
    shared.dashboard_rtl.m4            = 0.0f;
  }

  void navTick(float navDt) override {
    RawGpsReading gps;
    Trip_RTL      trip;
    float         headingDeg;
    float         currentAlt;
    withMutex([&]() {
      gps        = shared.raw.gps;
      trip       = shared.trip_rtl;
      headingDeg = shared.raw.compassHeadingDeg;
      currentAlt = shared.raw.baroAltitudeFt;

      shared.dashboard_rtl.altitudeFt     = currentAlt;
      shared.dashboard_rtl.compassHeading = headingDeg;
      shared.dashboard_rtl.roll           = shared.raw.imu.gyroX;
      shared.dashboard_rtl.pitch          = shared.raw.imu.gyroY;
      shared.dashboard_rtl.yaw            = shared.raw.imu.gyroZ;
    });

    if (trip.state == RTL_CLIMB) {
      if (currentAlt >= RTL_ALTITUDE_FT - 2.0f) {
        logLine("[RTL] Climb complete. Returning to launch.");
        withMutex([&]() { shared.trip_rtl.state = RTL_RETURN; });
      }
    }
    else if (trip.state == RTL_RETURN) {
      float distM   = gpsDistanceMeters(gps.lat, gps.lon, trip.launchLat, trip.launchLon);
      float bearing = gpsBearing(gps.lat, gps.lon, trip.launchLat, trip.launchLon);

      if (distM < WAYPOINT_ACCEPT_RADIUS_M) {
        logLine("[RTL] Arrived over launch pad. Settling.");
        withMutex([&]() {
          shared.trip_rtl.state            = RTL_SETTLE;
          shared.trip_rtl.settleStartMs    = millis();
          shared.cruise_rtl.targetPitchDeg = 0.0f;
          shared.cruise_rtl.targetRollDeg  = 0.0f;
        });
        return;
      }

      float northM;
      float eastM;
      bearingToNorthEast(distM, bearing, northM, eastM);

      withMutex([&]() { shared.cruise_rtl.yawTargetHeading = bearing; });

      withMutex([&]() {
        shared.cruise_rtl.targetRollDeg  = motorController.eastNavigationCorrection(eastM, navDt);
        shared.cruise_rtl.targetPitchDeg = motorController.northNavigationCorrection(northM, navDt);
      });
    }
    else if (trip.state == RTL_SETTLE) {
      if (millis() - trip.settleStartMs >= 3000) {
        logLine("[RTL] Settle complete. Beginning landing.");
        transitionTo(PHASE_LANDING, REASON_RTL_COMPLETE);
      }
    }
  }

  void physicsTick(float dt) override {
    Cruise_RTL c;
    RawSensors r;
    withMutex([&]() {
      c = shared.cruise_rtl;
      r = shared.raw;
    });

    MotorMix mix = motorController.computeMotorMix(
      c.targetAltFt, c.targetRollDeg, c.targetPitchDeg, c.yawTargetHeading,
      r.baroAltitudeFt, r.imu.gyroX, r.imu.gyroY, r.compassHeadingDeg, r.imu.gyroZ, dt);

    writeMotorMix(mix);

    withMutex([&]() {
      shared.dashboard_rtl.m1              = mix.m1;
      shared.dashboard_rtl.m2              = mix.m2;
      shared.dashboard_rtl.m3              = mix.m3;
      shared.dashboard_rtl.m4              = mix.m4;
      shared.dashboard_rtl.baseThrottle    = mix.baseThrottle;
      shared.dashboard_rtl.rollCorrection  = mix.rollCorrection;
      shared.dashboard_rtl.pitchCorrection = mix.pitchCorrection;
    });
  }

  void writeTelemetry(JsonDocument& doc) override {
    Dashboard_RTL d;
    Cruise_RTL    c;
    Trip_RTL      t;
    withMutex([&]() {
      d = shared.dashboard_rtl;
      c = shared.cruise_rtl;
      t = shared.trip_rtl;
    });

    doc["targetFt"]           = c.targetAltFt;
    doc["m1"]                 = (int)(d.m1 * 100);
    doc["m2"]                 = (int)(d.m2 * 100);
    doc["m3"]                 = (int)(d.m3 * 100);
    doc["m4"]                 = (int)(d.m4 * 100);
    doc["baseThrottle"]       = (int)(d.baseThrottle * 100);
    doc["rollCorrection"]     = (int)(d.rollCorrection * 100);
    doc["pitchCorrection"]    = (int)(d.pitchCorrection * 100);
    doc["compassHeading"]     = d.compassHeading;
    doc["navActive"]          = true;
    doc["flightSecRemaining"] = t.armedAtMs > 0
      ? max(0L, (long)((MAX_FLIGHT_TIME_MS - (millis() - t.armedAtMs)) / 1000))
      : (long)(MAX_FLIGHT_TIME_MS / 1000);
  }
};
