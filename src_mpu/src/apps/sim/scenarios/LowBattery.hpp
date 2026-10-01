#pragma once

// ============================================================================
// LOW BATTERY (lowbatt) -- the WORLD: the pack is only 25% charged. The pilot
// flies the mission as usual; the drone doesn't know the pack is weak until
// its voltage reading drops, and must then land where it is.
// ============================================================================

#include "Scenario.hpp"

class LowBattery : public Scenario {
public:
  const char* name() const override { return "lowbatt"; }

  void setup(SimRig& rig) override { rig.world->setCharge(0.25f); }

  void tick(SimRig& rig) override { pilot_.fly(rig.radio, /*mission=*/true); }

private:
  Pilot pilot_;
};
