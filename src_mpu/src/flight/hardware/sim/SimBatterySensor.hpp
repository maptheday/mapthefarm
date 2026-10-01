#pragma once

// ============================================================================
// SIM BATTERY SENSOR -- the sim's IBatterySensor: reports the world's pack
// voltage (it drains, sags under load, and has the LiFe "cliff").
// ============================================================================

#include "flight/FlightIo.hpp"   // IBatterySensor
#include "SimWorld.hpp"

class SimBatterySensor : public IBatterySensor {
public:
  SimBatterySensor(SimWorld* world) : world_(world) {}
  float readPackVolts() override { return world_->truth().packVolts; }
private:
  SimWorld* world_;
};
