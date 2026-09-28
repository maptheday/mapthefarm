#pragma once

#include "../models/ControlTypes.hpp"
#include "../state/FlightSettings.hpp"   // settings().airframe.hoverThrottle
#include "PID.hpp"

// Control service: converts targets and measurements into motor commands.
// It owns the PID memory because integrals and previous errors belong to the
// controller, not to a sensor or flight-phase data structure.
class MotorController {
public:
  // Load the PID gains from the settings (airframe section of
  // flightsettings.json). Called once by fc::begin(). The gains are the
  // tuning: "how hard to push per unit of error" (Lesson 4.9 of the course).
  void configure(const FlightSettings& s) {
    altitudePID = pidFrom(s.airframe.altitudePid);   // feet of error -> throttle trim
    rollPID     = pidFrom(s.airframe.rollPid);       // degrees -> motor difference
    pitchPID    = pidFrom(s.airframe.pitchPid);
    yawPID      = pidFrom(s.airframe.yawPid);
    // Fed the nav error in the drone's OWN frame (forward/right metres, see
    // northEastToForwardRight in NavMath.hpp): forward -> pitch, right -> roll.
    navForwardPID = pidFrom(s.airframe.navPid);
    navRightPID   = pidFrom(s.airframe.navPid);
    yawRateDamping_ = s.airframe.yawRateDamping;
    yawLimit_       = s.airframe.yawPid.max;
  }

  void reset() {
    altitudePID.reset();
    rollPID.reset();
    pitchPID.reset();
    yawPID.reset();
    navForwardPID.reset();
    navRightPID.reset();
  }

  // error = metres to go AHEAD of the nose -> target pitch (deg)
  float forwardNavigationCorrection(float error, float dt) {
    return navForwardPID.computeWithError(error, dt);
  }

  // error = metres to go to the drone's RIGHT -> target roll (deg)
  float rightNavigationCorrection(float error, float dt) {
    return navRightPID.computeWithError(error, dt);
  }

  MotorMix computeMotorMix(float targetAltFt, float targetRollDeg,
                           float targetPitchDeg, float yawTargetHeading,
                           float altFt, float roll, float pitch,
                           float compassHeading, float yawRateDps, float dt) {
    MotorMix out;
    // Hover feed-forward + PID trim: the baseline throttle holds the drone up,
    // the PID only corrects the error around it (much less integral windup and
    // altitude hunting than making the integral supply all of hover from zero).
    out.baseThrottle = settings().airframe.hoverThrottle + altitudePID.compute(targetAltFt, altFt, dt);

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

    // Rate damping: push against how fast we're already turning. This needs a
    // RATE (deg/s, + = heading increasing), not the fused yaw angle -- feeding it
    // the angle made the push scale with which way the nose pointed. The total is
    // capped at the yaw PID's own limit (airframe.yawPid.max) so yaw can never starve roll
    // and pitch of motor range.
    float baseYawCorrection = yawPID.computeWithError(yawError, dt);
    float yawCorrection = constrain(baseYawCorrection - (yawRateDps * yawRateDamping_),
                                    -yawLimit_, yawLimit_);

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
  static PID pidFrom(const PidGains& g) { return PID(g.kp, g.ki, g.kd, g.min, g.max); }

  // Placeholders until configure() loads the real gains from the settings.
  PID altitudePID{0, 0, 0, 0, 0};
  PID rollPID{0, 0, 0, 0, 0};
  PID pitchPID{0, 0, 0, 0, 0};
  PID yawPID{0, 0, 0, 0, 0};
  PID navForwardPID{0, 0, 0, 0, 0};
  PID navRightPID{0, 0, 0, 0, 0};
  float yawRateDamping_ = 0.0f;
  float yawLimit_       = 0.0f;
};

// The one MotorController instance (defined in FlightController.hpp).
extern MotorController motorController;