#pragma once

// ============================================================================
// MISSION -- the route the MISSION phase flies.
// The flight controller has no built-in route: it starts with the one in
// flightsettings.json (fc::begin), and an app can swap in a different one with
// fc::setMission() while the drone is on the ground (e.g. the first_mission app's
// short out-and-back).
// ============================================================================

#include <vector>
#include "../state/FlightSettings.hpp"   // Waypoint

inline std::vector<Waypoint>& missionRoute() {
  static std::vector<Waypoint> route;
  return route;
}
