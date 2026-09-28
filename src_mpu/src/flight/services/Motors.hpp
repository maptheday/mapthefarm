#pragma once

// ============================================================================
// MOTORS service -- the ONLY thing the phases use to drive the 4 motors.
// A phase computes a MotorMix and hands it here; this forwards it to the
// motors plug (IMotors: the real ESCs, or the sim's physics) and remembers it.
// Nothing else touches motors.
// ============================================================================

#include <Arduino.h>
#include "../models/ControlTypes.hpp"  // MotorMix
#include "../FlightIo.hpp"             // flightIo()

class Motors {
public:
  // Push one computed mix to all 4 motors.
  void writeMix(const MotorMix& mix) {
    lastMix = mix;
    if (IMotors* m = flightIo().motors) m->write(mix);
  }

  // Cut all motors immediately (the "stopped" signal).
  void disarmAll() {
    lastMix = MotorMix{};
    if (IMotors* m = flightIo().motors) m->stop();
  }

  MotorMix lastMix{};   // the last mix commanded: the battery's current estimate uses it
};

// The one Motors instance (defined in FlightController.hpp).
extern Motors motors;
