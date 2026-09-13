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

    float northM;
    float eastM;
    bearingToNorthEast(distM, bearing, northM, eastM);

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

  void writeTelemetry(JsonDocument& doc) override {
    Dashboard_Mission d;
    Cruise_Mission    c;
    Trip_Mission      t;
    withMutex([&]() {
      d = shared.dashboard_mission;
      c = shared.cruise_mission;
      t = shared.trip_mission;
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
    doc["gpsFix"]             = d.gpsFix;
    doc["gpsLat"]             = d.gpsLat;
    doc["gpsLon"]             = d.gpsLon;
    doc["gpsSats"]            = d.gpsSats;
    doc["navActive"]          = t.active;
    doc["navWaypoint"]        = t.currentWP;
    doc["navWaypointCount"]   = t.waypointCount;
    doc["navDistM"]           = d.distToWP;
    doc["navBearing"]         = d.bearingToWP;
    doc["flightSecRemaining"] = t.armedAtMs > 0
      ? max(0L, (long)((MAX_FLIGHT_TIME_MS - (millis() - t.armedAtMs)) / 1000))
      : (long)(MAX_FLIGHT_TIME_MS / 1000);
  }
};
