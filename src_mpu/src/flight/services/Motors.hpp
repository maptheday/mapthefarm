#pragma once

// ============================================================================
// MOTORS service -- the ONLY thing the phases use to drive the 4 motors.
// A phase computes a MotorMix and hands it here; this forwards it to the
// FlightIo (real ESCs, or the sim's physics). Nothing else touches motors.
// ============================================================================

#include <Arduino.h>
#include "../models/ControlTypes.hpp"  // MotorMix
#include "../FlightIo.hpp"             // FlightIo

class Motors {
public:
  // Called once by fc::begin(): where motor commands should go.
  void attach(FlightIo& io) { io_ = &io; }

  // Push one computed mix to all 4 motors.
  void writeMix(const MotorMix& mix) {
    lastMix = mix;
    if (io_) io_->writeMotors(mix);
  }

  // Cut all motors immediately (the "stopped" signal).
  void disarmAll() {
    lastMix = MotorMix{};
    if (io_) io_->stopMotors();
  }

  // Bench helpers -- motor is 1-based (M1..M4), PROPS OFF.
  void writeOne(int motor, float throttle) { if (io_) io_->writeMotor(motor, throttle); }
  void disarmOne(int motor)                { if (io_) io_->stopMotor(motor); }

  MotorMix lastMix{};   // the last mix commanded: the battery's current estimate
                        // uses it, and the sim's physics flies on it

private:
  FlightIo* io_ = nullptr;
};

// The one Motors instance (defined in FlightController.hpp).
extern Motors motors;
