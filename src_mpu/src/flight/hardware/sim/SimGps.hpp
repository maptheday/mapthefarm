#pragma once

// ============================================================================
// SIM GPS -- the sim's IGps: turns the world's TRUE position (meters from
// home) into lat/lon, always with a fix. (The gpsloss scenario wraps it to
// break it -- see apps/sim/scenarios/GpsLoss.hpp.)
// ============================================================================

#include "flight/FlightIo.hpp"   // IGps
#include "SimWorld.hpp"

class SimGps : public IGps {
public:
  SimGps(SimWorld* world) : world_(world) {}
  RawGpsReading read() override {
    Truth t = world_->truth();
    RawGpsReading r;
    r.lat       = t.lat;
    r.lon       = t.lon;
    r.fix       = true;
    r.sats      = 10;
    r.lastFixMs = millis();
    return r;
  }
private:
  SimWorld* world_;
};
