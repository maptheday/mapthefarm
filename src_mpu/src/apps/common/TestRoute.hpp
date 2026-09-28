#pragma once

// ============================================================================
// The short first-flight route: 30 m out along the FIRST leg of the configured
// mission route, then straight back, at 20 ft. Under a minute of flying and
// never far away. Used by the first_mission app (and the sim's "testroute"
// scenario, which checks it).
//
// It assumes the configured route ends back at the launch point (like the
// field route: "... then back to corner 1"), so the first leg starts there.
// ============================================================================

#include <vector>
#include "../../flight/state/FlightSettings.hpp"   // Waypoint
#include "../../flight/services/NavMath.hpp"       // gpsDistanceMeters

inline std::vector<Waypoint> makeTestRoute(const std::vector<Waypoint>& route,
                                           float outM = 30.0f, float altFt = 20.0f) {
  if (route.size() < 2) return route;            // nothing sensible to shorten
  const Waypoint& launch = route.back();         // the route ends where it started
  const Waypoint& first  = route.front();        // end of the first leg
  float legM = gpsDistanceMeters(launch.lat, launch.lon, first.lat, first.lon);
  float f    = legM > outM ? outM / legM : 1.0f; // how far along the leg to go

  Waypoint out  = { launch.lat + (first.lat - launch.lat) * f,
                    launch.lon + (first.lon - launch.lon) * f, altFt };
  Waypoint back = { launch.lat, launch.lon, altFt };
  return { out, back };
}
