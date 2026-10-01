#pragma once

// ============================================================================
// PWM MOTORS -- the real drone's IMotors: the 4 ESCs, driven with PWM (see
// EspPwmESC.hpp). Pins and pulse rate come from settings().wiring.
// ============================================================================

#include <Arduino.h>
#include "../../FlightIo.hpp"               // IMotors
#include "../../state/FlightSettings.hpp"   // settings().wiring.motorPins / escPwmHz
#include "../../services/Log.hpp"           // logLine
#include "EspPwmESC.hpp"

class PwmMotors : public IMotors {
public:
  void begin() override {
    const FlightSettings& s = settings();
    for (int i = 0; i < 4; i++) esc_[i].init(s.wiring.motorPins[i], i, s.wiring.escPwmHz);
    stop();
    logLine("[ESC] PWM channels initialized, all motors stopped.");
  }

  void write(const MotorMix& mix) override {
    esc_[0].write(mix.m1);
    esc_[1].write(mix.m2);
    esc_[2].write(mix.m3);
    esc_[3].write(mix.m4);
  }
  void stop() override { for (int i = 0; i < 4; i++) esc_[i].disarm(); }
  void writeOne(int motor, float throttle) override { esc_[motor - 1].write(throttle); }
  void stopOne(int motor) override { esc_[motor - 1].disarm(); }

  // Raw pulse width to every ESC -- only for the esc_calibration app.
  void writeAllMicroseconds(int us) { for (int i = 0; i < 4; i++) esc_[i].writeMicroseconds(us); }

private:
  EspPwmESC esc_[4];   // index 0 = M1 ... 3 = M4
};
