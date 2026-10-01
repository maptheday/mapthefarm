#pragma once

// ============================================================================
// MOTORS service -- the ONLY thing the phases use to drive the 4 motors.
// A phase computes a MotorMix and hands it here; this forwards it to the
// motors plug (IMotors: the real ESCs, or the sim's physics) and remembers it.
// Nothing else touches motors.
// ============================================================================

#include <Arduino.h>
#include "../models/ControlTypes.hpp"  // MotorMix
#include "../FlightIo.hpp"             // IMotors

class Motors {
public:
  Motors(IMotors* out) : out_(out) {}   // where the commands go

  // Push one computed mix to all 4 motors.
  void writeMix(const MotorMix& mix) {
    lastMix = mix;
    out_->write(mix);
  }

  // Cut all motors immediately (the "stopped" signal).
  void disarmAll() {
    lastMix = MotorMix{};
    out_->stop();
  }

  MotorMix lastMix{};   // the last mix commanded: the battery's current estimate uses it

private:
  IMotors* out_;
};
