#pragma once

// ============================================================
//  PID Controller
//
//  Three instincts working together:
//    P — how far off are we right now?
//    I — how long have we been slightly off?
//    D — are we closing in too fast and about to overshoot?
//
//  Usage:
//    PID pid(kp, ki, kd, outputMin, outputMax);
//    float output = pid.compute(target, actual, dt);
// ============================================================

class PID {
public:
  PID(float kp, float ki, float kd, float outMin, float outMax)
    : _kp(kp), _ki(ki), _kd(kd), _outMin(outMin), _outMax(outMax) {}

  float compute(float target, float actual, float dt) {
    if (dt <= 0) return 0;

    float error = target - actual;

    // P — proportional to current gap
    float p = _kp * error;

    // I — accumulated error over time (with windup clamp)
    _integral += error * dt;
    _integral = clamp(_integral, _outMin / _ki, _outMax / _ki);
    float i = _ki * _integral;

    // D — rate of change of error (how fast gap is closing)
    float derivative = (error - _lastError) / dt;
    float d = _kd * derivative;

    _lastError = error;

    return clamp(p + i + d, _outMin, _outMax);
  }

  float computeWithError(float error, float dt) {
    if (dt <= 0) return 0;

    // P
    float p = _kp * error;

    // I
    _integral += error * dt;
    _integral = clamp(_integral, _outMin / _ki, _outMax / _ki);
    float i = _ki * _integral;

    // D
    float derivative = (error - _lastError) / dt;
    float d = _kd * derivative;

    _lastError = error;

    return clamp(p + i + d, _outMin, _outMax);
  }

  void reset() {
    _integral  = 0;
    _lastError = 0;
  }

private:
  float _kp, _ki, _kd;
  float _outMin, _outMax;
  float _integral  = 0;
  float _lastError = 0;

  float clamp(float val, float lo, float hi) {
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
  }
};
