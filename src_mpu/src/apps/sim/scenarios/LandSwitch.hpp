#pragma once

// ============================================================================
// LAND SWITCH (land) -- the PILOT: take off, hover for 2 s, then flip LAND.
// The drone must land right where it is.
// ============================================================================

#include "Scenario.hpp"

class LandSwitch : public Scenario {
public:
  const char* name() const override { return "land"; }

  void tick(SimRig& rig) override {
    pilot_.fly(rig.radio, /*mission=*/false);
    if (!flipped_ && pilot_.hoveringMs() > 2000) {
      rig.radio.pressLand();
      flipped_ = true;
      logLine("[SCENARIO] LAND switch flipped");
    }
  }

private:
  Pilot pilot_;
  bool  flipped_ = false;
};
