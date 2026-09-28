#pragma once

// ============================================================================
// GPS LOSS (gpsloss) -- a broken SENSOR: once the drone is hovering, its GPS
// stops reporting a fix. The world carries on as normal (the drone is still
// wherever it is); only the drone's view of it is gone. With no position it
// can't fly home, so it must land where it is.
// ============================================================================

#include "Scenario.hpp"

// Wraps a GPS: works normally until lose() is called, then never has a fix.
class LosableGps : public IGps {
public:
  LosableGps(IGps* inner) : inner_(inner) {}   // the GPS it wraps
  void lose() { lost_ = true; }

  RawGpsReading read() override {
    RawGpsReading r = inner_->read();
    if (lost_) { r.fix = false; r.sats = 0; }
    return r;
  }

private:
  IGps* inner_;
  volatile bool lost_ = false;
};

class GpsLoss : public Scenario {
public:
  const char* name() const override { return "gpsloss"; }

  void setup(SimRig& rig) override {
    // C#:  gps = new LosableGps(gps);
    gps_ = new LosableGps(rig.io.gps);   // wrap the working GPS...
    rig.io.gps = gps_;                   // ...and give the flight controller the wrapper
  }

  void tick(SimRig& rig) override {
    pilot_.fly(rig.radio, /*mission=*/false);
    if (!lost_ && fc::phase() == PHASE_HOLD) {
      gps_->lose();
      lost_ = true;
      logLine("[SCENARIO] GPS fix dropped");
    }
  }

private:
  LosableGps* gps_ = nullptr;
  Pilot       pilot_;
  bool        lost_ = false;
};
