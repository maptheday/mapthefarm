#pragma once

// ============================================================================
// I2C BUS -- the two wires the IMU, compass and barometer share. Each of those
// drivers calls startI2c() in its begin(); only the first call does anything.
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include "../state/FlightSettings.hpp"   // settings().wiring.i2cSda / i2cScl

inline void startI2c() {
  static bool started = false;
  if (started) return;
  Wire.begin(settings().wiring.i2cSda, settings().wiring.i2cScl);
  started = true;
}
