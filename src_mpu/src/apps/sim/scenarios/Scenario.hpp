#pragma once

// ============================================================================
// SCENARIO -- one sim test. Each scenario is its own file in this folder, and
// can only do three kinds of thing:
//
//   1. change the WORLD        a gust, a weak pack          (rig.world->...)
//   2. break a SENSOR          wrap one of the plugs in     (rig.io, in setup())
//                              something that misbehaves
//   3. be the PILOT            flip switches, move sticks   (rig.radio->..., in tick())
//
// Rule changes (a smaller geofence, a shorter time limit, another route) are
// settings, so they live in override files instead:
//   data/flightsettings/sim.<scenario name>.json   picked up automatically
//   settingsFiles()                                any other file, e.g. the
//                                                  real first_mission.json
// ============================================================================

#include <Arduino.h>
#include <vector>
#include "flight/FlightController.hpp"   // fc::phase(), FlightIo
#include "../SimWorld.hpp"
#include "../SimRadio.hpp"

// What a scenario gets to work with.
struct SimRig {
  SimWorld* world;   // what's true
  SimRadio* radio;   // what the pilot holds
  FlightIo  io;      // the plugs the flight controller will get (setup() may swap one)
};

class Scenario {
public:
  virtual ~Scenario() {}
  virtual const char* name() const = 0;

  // Extra settings override files, loaded before sim.json.
  virtual std::vector<String> settingsFiles() const { return {}; }

  // Once, before the drone powers up: change the world, or wrap a plug.
  virtual void setup(SimRig&) {}

  // 10 times a second, while the sim runs: the pilot.
  virtual void tick(SimRig&) = 0;
};

// The pilot's usual routine: wait 2.5 s after power-up, flip START to take
// off, and (if `mission`) flip START again once it's hovering, to start the
// mission. Tries again every 1.5 s until the drone reacts.
class Pilot {
public:
  void fly(SimRadio* radio, bool mission) {
    unsigned long now = millis();
    if (t0Ms_ == 0) t0Ms_ = now;
    FlightPhase phase = fc::phase();

    if (phase == PHASE_HOLD) { if (holdSinceMs_ == 0) holdSinceMs_ = now; }
    else holdSinceMs_ = 0;

    if (now - t0Ms_ > 2500 && now - lastPressMs_ > 1500 &&
        (phase == PHASE_PARKED || (phase == PHASE_HOLD && mission))) {
      radio->pressStart();
      lastPressMs_ = now;
    }
  }

  // How long the drone has been hovering in HOLD (0 if it isn't).
  unsigned long hoveringMs() const { return holdSinceMs_ ? millis() - holdSinceMs_ : 0; }

private:
  unsigned long t0Ms_ = 0, lastPressMs_ = 0, holdSinceMs_ = 0;
};
