#pragma once

// ============================================================================
// BME280 ALTIMETER -- the real drone's IAltimeter: height above the launch
// point, in feet. Owns the barometer driver and the ground reference: begin()
// samples the ground pressure for ~2s and locks it in, so readFt() reports
// height ABOVE that spot rather than absolute altitude.
// ============================================================================

#include <Arduino.h>
#include "../../FlightIo.hpp"      // IAltimeter
#include "../../services/Log.hpp"  // logLine
#include "EspBarometer.hpp"
#include "I2cBus.hpp"           // startI2c

class Bme280Altimeter : public IAltimeter {
public:
  void begin() override {
    startI2c();
    baro_.initialize();
    // Average 20 readings over ~2s so the sensor settles and temperature
    // effects smooth out before we lock in the ground reference.
    logLine("[BARO] Sampling ground altitude (20 readings)...");
    const int SAMPLES     = 20;
    const int INTERVAL_MS = 100;
    float accum = 0.0f;
    for (int i = 0; i < SAMPLES; i++) {
      accum += (float)baro_.readAltitudeMeters() * 3.28084f;
      delay(INTERVAL_MS);
    }
    groundFt_ = accum / SAMPLES;
    logLine(String("[BARO] Ground altitude locked: ") + String(groundFt_, 1) + " ft");
  }

  // Feet above the locked-in ground reference.
  float readFt() override {
    return (float)baro_.readAltitudeMeters() * 3.28084f - groundFt_;
  }

private:
  EspBarometer baro_;
  float        groundFt_ = 0.0f;
};
