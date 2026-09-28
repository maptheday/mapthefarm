#pragma once

// ============================================================================
// HARDWARE IO -- the real drone's FlightIo: real sensors in, real ESCs out.
//
// Everything it needs (pins, ESC pulse rate, battery divider) comes from the
// settings (the "wiring" and "battery" sections of flightsettings.json).
//
// It comes up in three parts so small apps can use just what they need:
//   beginMotors()    the 4 ESCs            (esc_calibration uses only this)
//   beginSensors()   IMU, compass, GPS, barometer, battery pin
//   beginRadio()     the CRSF/ELRS receiver task
// begin() (what fc::begin() calls) does all three.
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include "../FlightIo.hpp"
#include "../state/FlightSettings.hpp"
#include "../hardware/EspPwmESC.hpp"
#include "../services/Imu.hpp"
#include "../services/Compass.hpp"
#include "../services/Gps.hpp"
#include "../services/Altimeter.hpp"
#include "../services/RcInput.hpp"
#include "../services/Log.hpp"

class HardwareIo : public FlightIo {
public:
  void begin() override {
    beginMotors();
    beginSensors();
    beginRadio();
  }

  void beginMotors() {
    const FlightSettings& s = settings();
    for (int i = 0; i < 4; i++) esc_[i].init(s.wiring.motorPins[i], i, s.wiring.escPwmHz);
    stopMotors();
    logLine("[ESC] PWM channels initialized, all motors stopped.");
  }

  void beginSensors() {
    const FlightSettings& s = settings();
    Wire.begin(s.wiring.i2cSda, s.wiring.i2cScl);
    imu.begin();
    compass.begin();
    logLine("[COMPASS] QMC5883L ready.");
    gps.begin();
    analogSetPinAttenuation(s.wiring.batteryAdc, ADC_11db);   // measure up to ~3 V on the pin
    altimeter.begin();   // samples + locks in "ground = 0 ft" -- keep the drone still
  }

  void beginRadio() {
    xTaskCreatePinnedToCore(crsfTask, "CRSFTask", 4096, NULL, 1, NULL, 0);
  }

  // --- inputs ---------------------------------------------------------------

  void readFast(float dt, FastInputs& in) override {
    imu.read(in.imu, dt);
    in.baroAltitudeFt = altimeter.readAltitudeFt();
    in.packVolts      = readPackVolts();
  }

  void readSlow(SlowInputs& in) override {
    RawGpsReading g;
    if (gps.read(g)) in.gps = g;
    else             in.gps.fix = false;
    in.compassHeadingDeg = compass.readHeadingDeg();
  }

  // The battery through the voltage divider (0 if nothing is wired).
  float readPackVolts() {
    return analogReadMilliVolts(settings().wiring.batteryAdc) / 1000.0f *
           settings().battery.dividerScale;
  }

  // --- motors ---------------------------------------------------------------

  void writeMotors(const MotorMix& mix) override {
    esc_[0].write(mix.m1);
    esc_[1].write(mix.m2);
    esc_[2].write(mix.m3);
    esc_[3].write(mix.m4);
  }
  void writeMotor(int motor, float throttle) override { esc_[motor - 1].write(throttle); }
  void stopMotors() override { for (int i = 0; i < 4; i++) esc_[i].disarm(); }
  void stopMotor(int motor) override { esc_[motor - 1].disarm(); }

  // Raw pulse width to every ESC -- only for the esc_calibration app.
  void writeAllMicroseconds(int us) { for (int i = 0; i < 4; i++) esc_[i].writeMicroseconds(us); }

private:
  EspPwmESC esc_[4];   // index 0 = M1 ... 3 = M4
};
