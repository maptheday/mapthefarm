#pragma once

// ============================================================================
// FLY MISSION -- nothing goes wrong: take off, fly the mission, land. The same
// pilot flies several scenarios, which differ only in their settings files:
//   full        the whole fence line
//   geofence    + sim.geofence.json   a 40 m fence -> the geofence failsafe (RTL)
//   timeout     + sim.timeout.json    an 18 s limit -> lands where it is
//   testroute   + first_mission.json  the short first-flight route
// ============================================================================

#include "Scenario.hpp"

class FlyMission : public Scenario {
public:
  FlyMission(const char* name, std::vector<String> files = {}) : name_(name), files_(files) {}
  const char* name() const override { return name_; }
  std::vector<String> settingsFiles() const override { return files_; }

  void tick(SimRig& rig) override { pilot_.fly(rig.radio, /*mission=*/true); }

private:
  const char*         name_;
  std::vector<String> files_;
  Pilot               pilot_;
};
