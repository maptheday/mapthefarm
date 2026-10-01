#pragma once

// ============================================================================
// SIM ALTIMETER -- the sim's IAltimeter: reports the world's TRUE height
// above the launch point.
// ============================================================================

#include "flight/FlightIo.hpp"   // IAltimeter
#include "SimWorld.hpp"

class SimAltimeter : public IAltimeter {
public:
  SimAltimeter(SimWorld* world) : world_(world) {}
  float readFt() override { return world_->truth().upFt; }
private:
  SimWorld* world_;
};
