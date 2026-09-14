#pragma once

// ============================================================================
// MANUAL -- the pilot flies with the RC sticks. This is the same machine as
// the autonomous phases: it only changes WHERE the Cruise setpoints come from.
// Instead of GPS/waypoint math, the sticks drive them (altitude-hold style):
//   throttle stick   -> climb/descend the target altitude (center = hold)
//   roll / pitch     -> target lean angle
//   yaw              -> turn the target heading
// physicsTick is the identical PID + motor mix every flying phase uses.
//
// POSITION HOLD (MANUAL_POSITION_HOLD): when you center the roll/pitch sticks,
// it drops a GPS "anchor" and actively leans back toward it so the drone stops
// drifting instead of coasting -- reusing the same nav PIDs MISSION/RTL use.
// Turn it off for bench/indoor testing (no GPS): a centered stick just levels.
//
// Entered/left with the RC's MANUAL switch (see RcInput). STOP always wins.
// ============================================================================

#include "IFlightPhase.hpp"
#include "../state/PhaseState.hpp"
#include "../state/FlightConfig.hpp"        // MANUAL_* tuning
#include "../services/Motors.hpp"           // motors
#include "../services/MotorController.hpp"  // motorController
#include "../services/NavMath.hpp"          // gpsDistanceMeters, gpsBearing, bearingToNorthEast

class ManualPhase : public IFlightPhase {
public:
  FlightPhase id() const override { return PHASE_MANUAL; }

  void onEnter(const EnterContext& ctx) override {
    // Start from "hold exactly where we are, level" so taking control doesn't
    // jolt the drone; the sticks move the setpoints from here.
    motorController.reset();
    shared.cruise_manual.targetAltFt      = ctx.currentAltFt;
    shared.cruise_manual.yawTargetHeading = ctx.currentHeadingDeg;
    shared.cruise_manual.targetRollDeg    = 0.0f;
    shared.cruise_manual.targetPitchDeg   = 0.0f;
    shared.trip_manual.anchored           = false;  // no anchor until sticks are centered
    shared.dashboard_manual.m1            = 0.0f;
    shared.dashboard_manual.m2            = 0.0f;
    shared.dashboard_manual.m3            = 0.0f;
    shared.dashboard_manual.m4            = 0.0f;
  }

  void navTick(float navDt) override {
    RawSticks     s;
    RawGpsReading gps;
    Trip_Manual   trip;
    withMutex([&]() {
      s    = shared.sticks;
      gps  = shared.raw.gps;
      trip = shared.trip_manual;
      shared.dashboard_manual.altitudeFt     = shared.raw.baroAltitudeFt;
      shared.dashboard_manual.compassHeading = shared.raw.compassHeadingDeg;
      shared.dashboard_manual.roll           = shared.raw.imu.gyroX;
      shared.dashboard_manual.pitch          = shared.raw.imu.gyroY;
      shared.dashboard_manual.yaw            = shared.raw.imu.gyroZ;
    });

    // throttle: 0..1 with 0.5 centered -> -1..1 deflection (up = climb).
    float climb   = deadband((s.throttle - 0.5f) * 2.0f) * MANUAL_CLIMB_RATE_FPS;
    float yawStep = deadband(s.yaw) * MANUAL_YAW_RATE_DPS * navDt;

    // Roll/pitch: either the pilot leans by hand, or -- if the sticks are
    // centered and position hold is on -- we hold a GPS anchor so it stops
    // drifting instead of coasting.
    float rollStick      = deadband(s.roll);
    float pitchStick     = deadband(s.pitch);
    bool  sticksCentered = (rollStick == 0.0f && pitchStick == 0.0f);

    float targetRoll;
    float targetPitch;
    if (MANUAL_POSITION_HOLD && sticksCentered && gps.fix) {
      // First centered tick: drop the anchor right where we let go.
      if (!trip.anchored) {
        trip.anchorLat = gps.lat;
        trip.anchorLon = gps.lon;
        trip.anchored  = true;
      }
      // Lean back toward the anchor to cancel drift (same math as MISSION).
      float distM   = gpsDistanceMeters(gps.lat, gps.lon, trip.anchorLat, trip.anchorLon);
      float bearing = gpsBearing(gps.lat, gps.lon, trip.anchorLat, trip.anchorLon);
      float northM, eastM;
      bearingToNorthEast(distM, bearing, northM, eastM);
      targetRoll  = motorController.eastNavigationCorrection(eastM, navDt);
      targetPitch = motorController.northNavigationCorrection(northM, navDt);
    } else {
      // Manual lean (or hold disabled / no GPS fix): fly by the sticks and drop
      // the anchor, so we re-anchor to the new spot next time you center.
      targetRoll    = rollStick  * MANUAL_MAX_LEAN_DEG;
      targetPitch   = pitchStick * MANUAL_MAX_LEAN_DEG;
      trip.anchored = false;
    }

    withMutex([&]() {
      float newAlt = shared.cruise_manual.targetAltFt + climb * navDt;
      shared.cruise_manual.targetAltFt    = newAlt < 0.0f ? 0.0f : newAlt;
      shared.cruise_manual.targetRollDeg  = targetRoll;
      shared.cruise_manual.targetPitchDeg = targetPitch;
      float h = shared.cruise_manual.yawTargetHeading + yawStep;
      shared.cruise_manual.yawTargetHeading = fmodf(h + 360.0f, 360.0f);
      // Persist the anchor field-by-field (shared is volatile; a whole-struct
      // assignment can't bind to it).
      shared.trip_manual.anchorLat = trip.anchorLat;
      shared.trip_manual.anchorLon = trip.anchorLon;
      shared.trip_manual.anchored  = trip.anchored;
    });
  }

  void physicsTick(float dt) override {
    Cruise_Manual c;
    RawSensors    r;
    withMutex([&]() {
      c = shared.cruise_manual;
      r = shared.raw;
    });

    MotorMix mix = motorController.computeMotorMix(
      c.targetAltFt, c.targetRollDeg, c.targetPitchDeg, c.yawTargetHeading,
      r.baroAltitudeFt, r.imu.gyroX, r.imu.gyroY, r.compassHeadingDeg, r.imu.gyroZ, dt);

    motors.writeMix(mix);

    withMutex([&]() {
      shared.dashboard_manual.m1              = mix.m1;
      shared.dashboard_manual.m2              = mix.m2;
      shared.dashboard_manual.m3              = mix.m3;
      shared.dashboard_manual.m4              = mix.m4;
      shared.dashboard_manual.baseThrottle    = mix.baseThrottle;
      shared.dashboard_manual.rollCorrection  = mix.rollCorrection;
      shared.dashboard_manual.pitchCorrection = mix.pitchCorrection;
    });
  }

private:
  // Ignore tiny stick noise near center so the drone doesn't creep.
  static float deadband(float v) {
    return (v > -MANUAL_STICK_DEADBAND && v < MANUAL_STICK_DEADBAND) ? 0.0f : v;
  }
};
