#pragma once

// ============================================================================
// MOTORS service -- the ONLY thing that talks to the 4 ESCs.
// A phase computes a MotorMix and hands it here; this forwards it to the PWM
// motors. Nothing else in the code touches motor pins directly.
//
// Under SIM there is no motor hardware, so every method is a no-op --
// the HIL tests exercise the flight logic, not the ESC wiring.
//
// ESCs speak standard PWM (EspPwmESC.hpp), which every ESC understands.
// ============================================================================

#include <Arduino.h>
#include "../hardware/EspPwmESC.hpp"
#include "../models/ControlTypes.hpp"  // MotorMix
#include "Log.hpp"                     // logLine

class Motors {
public:
  // Bring up each motor's PWM channel and send "stopped" to all 4.
  void begin() {
#ifndef SIM
    const int pins[4] = { 4, 5, 6, 7 };   // M1..M4
    for (int i = 0; i < 4; i++) esc_[i].init(pins[i], i);   // PWM channels 0..3
    disarmAll();
#endif
  }

  // Push one computed mix to all 4 motors.
  void writeMix(const MotorMix& mix) {
    lastMix = mix;   // read by the battery estimate (and, in SIM, the physics)
#ifndef SIM
    esc_[0].write(mix.m1);
    esc_[1].write(mix.m2);
    esc_[2].write(mix.m3);
    esc_[3].write(mix.m4);
#else
    (void)mix;
#endif
  }

  // Cut all motors immediately (the 1000 us "stopped" pulse).
  void disarmAll() {
    lastMix = MotorMix{};   // motors stopped
#ifndef SIM
    for (int i = 0; i < 4; i++) esc_[i].disarm();
#endif
  }

  // Bench test helpers -- motor is 1-based (M1..M4), PROPS OFF.
  void writeOne(int motor, float throttle) {
#ifndef SIM
    esc_[motor - 1].write(throttle);
#else
    (void)motor; (void)throttle;
#endif
  }
  void disarmOne(int motor) {
#ifndef SIM
    esc_[motor - 1].disarm();
#else
    (void)motor;
#endif
  }

  // One-time ESC calibration: teach all 4 ESCs that 2000 us = full and
  // 1000 us = stopped. Walks you through it over the serial monitor, then
  // halts forever so the drone can't fly in this mode. Run from setup() when
  // CALIBRATE_ESCS_ON_BOOT is true, BEFORE the flight tasks start.
  //
  // Why this order: an ESC checks the signal the moment it gets battery power.
  // If it sees FULL throttle then, it enters "learn the range" mode, beeps,
  // and remembers that width as full. Dropping to MIN then teaches it "stopped".
  void runEscCalibration() {
#ifndef SIM
    logLine("");
    logLine("=== ESC CALIBRATION ===");
    logLine("1. Take ALL PROPS OFF.");
    logLine("2. UNPLUG the flight battery (the ESP32 stays powered by USB).");
    logLine("Type GO and press Enter when both are done.");
    waitForWord("GO");

    for (int i = 0; i < 4; i++) esc_[i].writeMicroseconds(EspPwmESC::MAX_US);
    logLine("Sending FULL throttle signal (no power reaches the motors yet).");
    logLine("3. Plug in the flight battery now.");
    logLine("   The ESCs play a startup tune, then a short 'beep-beep'.");
    logLine("   Right after the beep-beep, type MIN and press Enter.");
    logLine("   (If the motors SPIN instead of beeping, unplug the battery at once.)");
    waitForWord("MIN");

    for (int i = 0; i < 4; i++) esc_[i].writeMicroseconds(EspPwmESC::MIN_US);
    logLine("Sending STOPPED signal.");
    logLine("4. The ESCs beep once per battery cell (3 for 3S), then a long beep:");
    logLine("   the range is saved.");
    logLine("5. Unplug the battery. Set CALIBRATE_ESCS_ON_BOOT back to false and flash again.");
    logLine("=== DONE. Halting (motors stay stopped). ===");
    while (true) delay(1000);   // never continue into flight in calibration mode
#endif
  }

  MotorMix lastMix{};   // last mix commanded: the battery's current estimate uses it,
                        // and in SIM the on-chip physics flies on it

private:
  EspPwmESC esc_[4]; // index 0=M1, 1=M2, 2=M3, 3=M4

  // Block until the given word is typed on the serial monitor (not case-sensitive).
  void waitForWord(const char* word) {
    String line;
    while (true) {
      while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
          line.trim();
          if (line.equalsIgnoreCase(word)) return;
          if (line.length() > 0) logLine(String("   (waiting for ") + word + ")");
          line = "";
        } else {
          line += c;
        }
      }
      delay(10);
    }
  }
};

// The one Motors instance (defined in the .ino). Include this header to use it.
extern Motors motors;
