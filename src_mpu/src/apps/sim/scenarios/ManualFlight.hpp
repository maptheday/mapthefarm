#pragma once

// ============================================================================
// MANUAL FLIGHT (manual) -- the PILOT: take off, flip MANUAL once hovering,
// push the pitch stick forward for 5 s, then center the sticks and hit STOP.
// The drone must fly where the sticks say.
// ============================================================================

#include "Scenario.hpp"

class ManualFlight : public Scenario {
public:
  const char* name() const override { return "manual"; }

  void tick(SimRig& rig) override {
    if (done_) return;
    if (!onSinceMs_) {
      pilot_.fly(rig.radio, /*mission=*/false);
      if (fc::phase() == PHASE_HOLD) {
        rig.radio->setManual(true);
        onSinceMs_ = millis();
        logLine("[SCENARIO] manual control on");
      }
      return;
    }
    if (millis() - onSinceMs_ < 5000) {
      rig.radio->setSticks(0.5f, 0.0f, 0.5f, 0.0f);   // hold height, pitch forward
    } else {
      rig.radio->setSticks(0.5f, 0.0f, 0.0f, 0.0f);   // center
      rig.radio->setStop(true);
      done_ = true;
      logLine("[SCENARIO] ===SCENARIO_DONE===");
    }
  }

private:
  Pilot         pilot_;
  unsigned long onSinceMs_ = 0;
  bool          done_ = false;
};
