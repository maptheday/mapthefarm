#pragma once

#include "../models/ControlTypes.hpp"
#include "../state/FlightConfig.hpp"   // HOVER_THROTTLE_FF
#include "PID.hpp"

// Control service: converts targets and measurements into motor commands.
// It owns the PID memory because integrals and previous errors belong to the
// controller, not to a sensor or flight-phase data structure.
class MotorController {
public:
  MotorController()
    // Altitude PID now trims AROUND the hover feed-forward, so its output is a
    // correction that can go negative (to descend) -- clamp +/-0.45, not 0..1.
    // P is deliberately gentle (0.02): the old 0.08 saturated the clamp at only
    // ~6 ft of error, turning height control into bang-bang that porpoised the
    // drone. This only surfaced once RotorPy closed the loop the HIL tests fake.
    : altitudePID(0.02f, 0.008f, 0.10f, -0.45f, 0.45f),
      rollPID(0.01f, 0.001f, 0.005f, -0.3f, 0.3f),
      pitchPID(0.01f, 0.001f, 0.005f, -0.3f, 0.3f),
      yawPID(0.005f, 0.0001f, 0.001f, -0.2f, 0.2f),
      // Gentle, well-damped GPS navigation: a ±6 deg tilt cap so it doesn't slam
      // to full lean over long legs (which built too much speed and made it
      // overshoot/orbit each waypoint), with strong D to brake on approach.
      navNorthPID(0.35f, 0.0f, 0.6f, -6.0f, 6.0f),
      navEastPID(0.35f, 0.0f, 0.6f, -6.0f, 6.0f) {}

  void reset() {
    altitudePID.reset();
    rollPID.reset();
    pitchPID.reset();
    yawPID.reset();
    navNorthPID.reset();
    navEastPID.reset();
  }

  float northNavigationCorrection(float error, float dt) {
    return navNorthPID.computeWithError(error, dt);
  }

  float eastNavigationCorrection(float error, float dt) {
    return navEastPID.computeWithError(error, dt);
  }

  MotorMix computeMotorMix(float targetAltFt, float targetRollDeg,
                           float targetPitchDeg, float yawTargetHeading,
                           float altFt, float roll, float pitch,
                           float compassHeading, float gyroZ, float dt) {
    MotorMix out;
    // Hover feed-forward + PID trim: the baseline throttle holds the drone up,
    // the PID only corrects the error around it (much less integral windup and
    // altitude hunting than making the integral supply all of hover from zero).
    out.baseThrottle = HOVER_THROTTLE_FF + altitudePID.compute(targetAltFt, altFt, dt);

    // Throttle tilt compensation: when the drone banks, only cos(tilt) of its
    // thrust points up, so it sinks unless we push harder. Scale base throttle by
    // 1/(cos roll * cos pitch) to hold altitude while maneuvering. Capped so an
    // extreme tilt (or a bad reading) can't command runaway thrust.
    float rollRad  = roll  * 0.0174532925f;
    float pitchRad = pitch * 0.0174532925f;
    float tiltFactor = 1.0f / (cosf(rollRad) * cosf(pitchRad));
    tiltFactor = constrain(tiltFactor, 1.0f, 2.0f);   // 2.0 ~= 60 deg of tilt
    out.baseThrottle = constrain(out.baseThrottle * tiltFactor, 0.0f, 1.0f);

    out.rollCorrection = rollPID.compute(targetRollDeg, roll, dt);
    out.pitchCorrection = pitchPID.compute(targetPitchDeg, pitch, dt);

    float yawError = yawTargetHeading - compassHeading;
    if (yawError > 180.0f) yawError -= 360.0f;
    if (yawError < -180.0f) yawError += 360.0f;

    float baseYawCorrection = yawPID.computeWithError(yawError, dt);
    float yawCorrection = baseYawCorrection - (gyroZ * 0.02f);

    out.m1 = constrain(out.baseThrottle + out.pitchCorrection +
                       out.rollCorrection - yawCorrection, 0.0f, 1.0f);
    out.m2 = constrain(out.baseThrottle + out.pitchCorrection -
                       out.rollCorrection + yawCorrection, 0.0f, 1.0f);
    out.m3 = constrain(out.baseThrottle - out.pitchCorrection +
                       out.rollCorrection + yawCorrection, 0.0f, 1.0f);
    out.m4 = constrain(out.baseThrottle - out.pitchCorrection -
                       out.rollCorrection - yawCorrection, 0.0f, 1.0f);
    return out;
  }

private:
  PID altitudePID;
  PID rollPID;
  PID pitchPID;
  PID yawPID;
  PID navNorthPID;
  PID navEastPID;
};

// The one MotorController instance (defined in the .ino).
extern MotorController motorController;