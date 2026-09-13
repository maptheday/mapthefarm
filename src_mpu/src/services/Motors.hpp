#pragma once

// ============================================================================
// MOTORS service -- the ONLY thing that talks to the 4 ESCs.
// A phase computes a MotorMix and hands it here; this forwards it to the DShot
// motors. Nothing else in the code touches motor pins directly.
//
// Under WOKWI_SIM there is no motor hardware, so every method is a no-op --
// the HIL tests exercise the flight logic, not the ESC wiring.
// ============================================================================

#include <Arduino.h>
#include "../hardware/EspESC.hpp"      // pulls in driver/rmt.h (RMT_CHANNEL_*)
#include "../models/ControlTypes.hpp"  // MotorMix

class Motors {
public:
  // Bring up each motor's RMT channel and confirm all are disarmed (at zero).
  // DShot600 has no calibration/arming beep sequence -- that's a PWM-ESC ritual.
  void begin() {
#ifndef WOKWI_SIM
    const int           pins[4] = { 4, 5, 6, 7 };            // M1..M4
    const rmt_channel_t ch[4]   = { RMT_CHANNEL_0, RMT_CHANNEL_1,
                                    RMT_CHANNEL_2, RMT_CHANNEL_3 };
    for (int i = 0; i < 4; i++) esc_[i].init(pins[i], ch[i]);
    disarmAll();
#endif
  }

  // Push one computed mix to all 4 motors.
  void writeMix(const MotorMix& mix) {
#ifndef WOKWI_SIM
    esc_[0].write(mix.m1);
    esc_[1].write(mix.m2);
    esc_[2].write(mix.m3);
    esc_[3].write(mix.m4);
#else
    (void)mix;
#endif
  }

  // Cut all motors immediately (DShot disarm command, not a PWM "min throttle").
  void disarmAll() {
#ifndef WOKWI_SIM
    for (int i = 0; i < 4; i++) esc_[i].disarm();
#endif
  }

  // Bench test helpers -- motor is 1-based (M1..M4), PROPS OFF.
  void writeOne(int motor, float throttle) {
#ifndef WOKWI_SIM
    esc_[motor - 1].write(throttle);
#else
    (void)motor; (void)throttle;
#endif
  }
  void disarmOne(int motor) {
#ifndef WOKWI_SIM
    esc_[motor - 1].disarm();
#else
    (void)motor;
#endif
  }

private:
  EspESC esc_[4]; // index 0=M1, 1=M2, 2=M3, 3=M4
};

// The one Motors instance (defined in the .ino). Include this header to use it.
extern Motors motors;
