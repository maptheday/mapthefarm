#pragma once

// ============================================================================
// STABILIZATION (stab) -- the WORLD: once the drone is hovering, a gust rolls
// it over ~22 deg for half a second. The controller must bring it back to
// level. After 6 s the pilot hits STOP.
// ============================================================================

#include "Scenario.hpp"

class Stabilization : public Scenario {
public:
  const char* name() const override { return "stab"; }

  void tick(SimRig& rig) override {
    if (done_) return;
    if (!gustMs_) {
      pilot_.fly(rig.radio, /*mission=*/false);
      if (fc::phase() == PHASE_HOLD) {
        rig.world.gust(22.0f, 500);
        gustMs_ = millis();
        logLine("[SCENARIO] attitude kick applied");
      }
      return;
    }
    if (millis() - gustMs_ > 6000) {
      rig.radio.setStop(true);
      done_ = true;
      logLine("[SCENARIO] ===SCENARIO_DONE===");
    }
  }

private:
  Pilot         pilot_;
  unsigned long gustMs_ = 0;
  bool          done_ = false;
};
