#pragma once

// ============================================================================
// SIM RADIO -- the sim's IRadio: a fake transmitter with the same sticks and
// switches as the real one. A scenario's pilot moves them; the radio sends a
// frame every 20 ms (50 Hz), and the flight controller treats those frames
// exactly like real CRSF frames: same switch logic, same "radio is alive"
// bookkeeping, same radio-loss failsafe.
// ============================================================================

#include <Arduino.h>
#include "flight/FlightIo.hpp"

class SimRadio : public IRadio {
public:
  static constexpr unsigned long FRAME_MS = 20;
  static constexpr unsigned long PRESS_MS = 300;   // how long a "flip" holds a switch up

  // --- the flight controller's side -----------------------------------------
  bool read(RadioFrame& out) override {
    unsigned long now = millis();
    if (now - lastFrameMs_ < FRAME_MS) return false;
    lastFrameMs_ = now;
    portENTER_CRITICAL(&mux_);
    out = frame_;
    out.start = now < startUntilMs_;
    out.land  = now < landUntilMs_;
    portEXIT_CRITICAL(&mux_);
    return true;
  }

  // --- the pilot's side -----------------------------------------------------

  // Flip START / LAND up, then (a moment later) back down.
  void pressStart() { portENTER_CRITICAL(&mux_); startUntilMs_ = millis() + PRESS_MS; portEXIT_CRITICAL(&mux_); }
  void pressLand()  { portENTER_CRITICAL(&mux_); landUntilMs_  = millis() + PRESS_MS; portEXIT_CRITICAL(&mux_); }

  // Switches that stay where you put them.
  void setManual(bool on) { portENTER_CRITICAL(&mux_); frame_.manual = on; portEXIT_CRITICAL(&mux_); }
  void setStop(bool on)   { portENTER_CRITICAL(&mux_); frame_.stop   = on; portEXIT_CRITICAL(&mux_); }

  // throttle 0..1 (0.5 = hold height); roll/pitch/yaw -1..1 (0 = centered).
  void setSticks(float throttle, float roll, float pitch, float yaw) {
    portENTER_CRITICAL(&mux_);
    frame_.sticks.throttle = throttle;
    frame_.sticks.roll     = roll;
    frame_.sticks.pitch    = pitch;
    frame_.sticks.yaw      = yaw;
    portEXIT_CRITICAL(&mux_);
  }

private:
  portMUX_TYPE  mux_ = portMUX_INITIALIZER_UNLOCKED;
  RadioFrame    frame_{};   // sticks centered, every switch down
  unsigned long startUntilMs_ = 0, landUntilMs_ = 0, lastFrameMs_ = 0;
};
