#pragma once

// ============================================================================
// SIM COMPASS -- the sim's ICompass: reports the world's TRUE heading.
// (Nothing to calibrate, so the calibration methods do nothing.)
// ============================================================================

#include "flight/FlightIo.hpp"   // ICompass
#include "SimWorld.hpp"

class SimCompass : public ICompass {
public:
  SimCompass(SimWorld* world) : world_(world) {}
  float readHeadingDeg() override { return world_->truth().headingDeg; }
private:
  SimWorld* world_;
};
