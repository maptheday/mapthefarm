#pragma once

// ============================================================================
// MISSION -- fly the waypoint list. Each nav tick: check failsafes, aim at the
// current waypoint (turn to face it, tilt to move toward it), and advance when
// close enough. After the last waypoint, go to HOVER_SETTLE.
// ============================================================================

#include "IFlightPhase.hpp"
#include "../state/PhaseState.hpp"
#include "../state/FlightConfig.hpp"        // WAYPOINTS, WAYPOINT_*, MAX_FLIGHT_TIME_MS
#include "../services/Motors.hpp"           // motors
#include "../services/MotorController.hpp"  // motorController
#include "../services/NavMath.hpp"          // gpsDistanceMeters, gpsBearing, getMissionWaypoint
#include "../services/Failsafes.hpp"        // checkCoreFailsafes
#include "../services/Log.hpp"              // logLine, PANIC
#include "PhaseSwitch.hpp"                  // transitionTo

class MissionPhase : public IFlightPhase {
public:
  FlightPhase id() const override { return PHASE_MISSION; }

  void onEnter(const EnterContext& ctx) override {
    if (WAYPOINT_COUNT == 0) { PANIC("transitionTo MISSION: zero waypoints"); }
    shared.trip_mission.armedAtMs          = ctx.carriedArmedAtMs;
    shared.trip_mission.launchLat          = ctx.carriedLaunchLat;
    shared.trip_mission.launchLon          = ctx.carriedLaunchLon;
    shared.trip_mission.currentWP          = 0;
    shared.trip_mission.waypointCount      = WAYPOINT_COUNT;
    shared.trip_mission.active             = true;
    shared.cruise_mission.targetAltFt      = WAYPOINTS[0].altFt;
    shared.cruise_mission.yawTargetHeading = ctx.currentHeadingDeg;
    shared.cruise_mission.targetRollDeg    = 0.0f;
    shared.cruise_mission.targetPitchDeg   = 0.0f;
    shared.dashboard_mission.m1            = 0.0f;
    shared.dashboard_mission.m2            = 0.0f;
    shared.dashboard_mission.m3            = 0.0f;
    shared.dashboard_mission.m4            = 0.0f;
  }

  void navTick(float navDt) override {
    RawGpsReading gps;
    Trip_Mission  trip;
    float         headingDeg;
    withMutex([&]() {
      gps        = shared.raw.gps;
      trip       = shared.trip_mission;
      headingDeg = shared.raw.compassHeadingDeg;

      shared.dashboard_mission.altitudeFt     = shared.raw.baroAltitudeFt;
      shared.dashboard_mission.compassHeading = headingDeg;
      shared.dashboard_mission.gpsLat         = gps.lat;
      shared.dashboard_mission.gpsLon         = gps.lon;
      shared.dashboard_mission.gpsFix         = gps.fix;
      shared.dashboard_mission.gpsSats        = gps.sats;
      shared.dashboard_mission.roll           = shared.raw.imu.gyroX;
      shared.dashboard_mission.pitch          = shared.raw.imu.gyroY;
      shared.dashboard_mission.yaw            = shared.raw.imu.gyroZ;
    });

    checkCoreFailsafes(trip.armedAtMs, trip.launchLat, trip.launchLon);
    withMutex([&]() { trip = shared.trip_mission; }); // reload in case a failsafe changed phase

    if (!trip.active) return;

    if (trip.currentWP >= trip.waypointCount) {
      transitionTo(PHASE_HOVER_SETTLE, REASON_MISSION_COMPLETE);
      logLine("[NAV] Mission complete — hovering before landing.");
      return;
    }

    Waypoint wp      = getMissionWaypoint(trip.currentWP, trip.launchLat, trip.launchLon);
    float    distM   = gpsDistanceMeters(gps.lat, gps.lon, wp.lat, wp.lon);
    float    bearing = gpsBearing(gps.lat, gps.lon, wp.lat, wp.lon);

    withMutex([&]() {
      shared.cruise_mission.yawTargetHeading = bearing;
      shared.dashboard_mission.distToWP      = distM;
      shared.dashboard_mission.bearingToWP   = bearing;
    });

    if (distM < WAYPOINT_ACCEPT_RADIUS_M) {
      logLine(String("[NAV] Reached waypoint ") + String(trip.currentWP) + " — advancing.");
      withMutex([&]() {
        shared.trip_mission.currentWP++;
        int next = shared.trip_mission.currentWP;
        shared.cruise_mission.targetAltFt = (next < shared.trip_mission.waypointCount)
            ? getMissionWaypoint(next, trip.launchLat, trip.launchLon).altFt : wp.altFt;
      });
      return;
    }

    // Line-following: steer toward a carrot that rides ALONG the leg from the
    // previous waypoint to this one, instead of aiming straight at the waypoint
    // (which lets the drone bow to the inside of the turn). The previous point is
    // the launch pad for the first leg, otherwise the prior waypoint.
    double prevLat, prevLon;
    if (trip.currentWP == 0) {
      prevLat = trip.launchLat;
      prevLon = trip.launchLon;
    } else {
      Waypoint prevWp = getMissionWaypoint(trip.currentWP - 1, trip.launchLat, trip.launchLon);
      prevLat = prevWp.lat;
      prevLon = prevWp.lon;
    }

    float northM;
    float eastM;
    lineFollowNorthEast(gps.lat, gps.lon, prevLat, prevLon, wp.lat, wp.lon,
                        WAYPOINT_LOOKAHEAD_M, northM, eastM);

    withMutex([&]() {
      shared.cruise_mission.targetRollDeg  = motorController.eastNavigationCorrection(eastM, navDt);
      shared.cruise_mission.targetPitchDeg = motorController.northNavigationCorrection(northM, navDt);
    });
  }

  void physicsTick(float dt) override {
    Cruise_Mission c;
    RawSensors     r;
    withMutex([&]() {
      c = shared.cruise_mission;
      r = shared.raw;
    });

    MotorMix mix = motorController.computeMotorMix(
      c.targetAltFt, c.targetRollDeg, c.targetPitchDeg, c.yawTargetHeading,
      r.baroAltitudeFt, r.imu.gyroX, r.imu.gyroY, r.compassHeadingDeg, r.imu.gyroZ, dt);

    motors.writeMix(mix);

    withMutex([&]() {
      shared.dashboard_mission.m1              = mix.m1;
      shared.dashboard_mission.m2              = mix.m2;
      shared.dashboard_mission.m3              = mix.m3;
      shared.dashboard_mission.m4              = mix.m4;
      shared.dashboard_mission.baseThrottle    = mix.baseThrottle;
      shared.dashboard_mission.rollCorrection  = mix.rollCorrection;
      shared.dashboard_mission.pitchCorrection = mix.pitchCorrection;
    });
  }
};
