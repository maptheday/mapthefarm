#pragma once

// ============================================================================
// RADIO LOSS (rcloss) -- a broken SENSOR: the radio link. The pilot starts the
// mission as usual; 10 s in, the receiver stops hearing the transmitter. With
// no radio, the pilot's STOP and LAND switches can't reach the drone, so the
// radio failsafe must bring it home (RTL) and land it.
// ============================================================================

#include "Scenario.hpp"

// Wraps a radio: works normally until cut() is called, then never hears a frame.
class CuttableRadio : public IRadio {
public:
  CuttableRadio(IRadio* inner) : inner_(inner) {}   // the radio it wraps
  void cut() { cut_ = true; }

  bool read(RadioFrame& out) override {
    RadioFrame f;
    bool got = inner_->read(f);   // the transmitter keeps sending...
    if (cut_) return false;      // ...but nothing arrives
    if (got) out = f;
    return got;
  }

private:
  IRadio* inner_;
  volatile bool cut_ = false;
};

class RadioLoss : public Scenario {
public:
  const char* name() const override { return "rcloss"; }

  void setup(SimRig& rig) override {
    // C#:  radio = new CuttableRadio(radio);
    radio_ = new CuttableRadio(rig.io.radio);   // wrap the working radio...
    rig.io.radio = radio_;                      // ...and give the flight controller the wrapper
  }

  void tick(SimRig& rig) override {
    pilot_.fly(rig.radio, /*mission=*/true);
    if (missionSinceMs_ == 0 && fc::phase() == PHASE_MISSION) missionSinceMs_ = millis();
    if (!cut_ && missionSinceMs_ && millis() - missionSinceMs_ > 10000) {
      radio_->cut();
      cut_ = true;
      logLine("[SCENARIO] radio link cut");
    }
  }

private:
  CuttableRadio* radio_ = nullptr;
  Pilot          pilot_;
  unsigned long  missionSinceMs_ = 0;
  bool           cut_ = false;
};
