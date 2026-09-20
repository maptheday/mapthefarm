#pragma once

// ============================================================================
// NAV MATH service -- pure "where am I vs. where do I want to be" geometry.
// No state, no hardware: give it two GPS points and it tells you how far apart
// they are, which way to point, and how that splits into north/east motion.
// ============================================================================

#include <Arduino.h>
#include "../state/FlightConfig.hpp"  // Waypoint, WAYPOINTS, WAYPOINT_COUNT

// Straight-line distance between two lat/lon points, in metres (haversine).
inline float gpsDistanceMeters(double lat1, double lon1, double lat2, double lon2) {
  const float R = 6371000.0f;
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  float a = sin(dLat/2)*sin(dLat/2) + cos(radians(lat1))*cos(radians(lat2))*sin(dLon/2)*sin(dLon/2);
  return R * 2.0f * atan2(sqrt(a), sqrt(1.0f - a));
}

// Compass bearing (0-360deg) you'd fly to get from point 1 to point 2.
inline float gpsBearing(double lat1, double lon1, double lat2, double lon2) {
  float dLon = radians(lon2 - lon1);
  float y    = sin(dLon) * cos(radians(lat2));
  float x    = cos(radians(lat1)) * sin(radians(lat2)) - sin(radians(lat1)) * cos(radians(lat2)) * cos(dLon);
  return fmod(degrees(atan2(y, x)) + 360.0f, 360.0f);
}

// Split "distance + bearing" into north and east components (metres).
inline void bearingToNorthEast(float distM, float bearingDeg, float& northM, float& eastM) {
  float rad = radians(bearingDeg);
  northM = distM * cos(rad);
  eastM  = distM * sin(rad);
}

// Line-following "carrot" target (L1-style). Instead of flying straight at the
// next waypoint -- which lets the drone cut to the INSIDE of each turn and bow
// off the straight leg -- steer toward a point that rides ALONG the segment from
// the previous waypoint to the current one, a fixed lookahead ahead of where the
// drone currently projects onto that segment. The drone then converges onto the
// line and hugs it corner-to-corner.
//
// Returns the north/east vector (metres) FROM the current position TO the carrot,
// in the same world N/E convention as bearingToNorthEast, so it drops straight
// into the nav PIDs. Falls back to aiming at the waypoint if the segment is tiny.
inline void lineFollowNorthEast(double curLat, double curLon,      // where we are
                                double prevLat, double prevLon,     // segment start
                                double wpLat, double wpLon,         // segment end (waypoint)
                                float lookaheadM,
                                float& northM, float& eastM) {
  // Local equirectangular metres, origin at the segment start.
  float mPerLat = 111320.0f;
  float mPerLon = 111320.0f * cos(radians(prevLat));
  float segE = (wpLon  - prevLon) * mPerLon;   // segment end
  float segN = (wpLat  - prevLat) * mPerLat;
  float posE = (curLon - prevLon) * mPerLon;   // current position
  float posN = (curLat - prevLat) * mPerLat;

  float segLen = sqrt(segE * segE + segN * segN);
  if (segLen < 1.0f) {                          // degenerate segment: aim at the wp
    northM = segN - posN;
    eastM  = segE - posE;
    return;
  }
  float uE = segE / segLen, uN = segN / segLen;         // unit vector along the leg
  float proj = posE * uE + posN * uN;                    // how far along the leg we are
  if (proj < 0.0f) proj = 0.0f;
  float carrot = proj + lookaheadM;                      // ride the line, lookahead ahead
  if (carrot > segLen) carrot = segLen;                  // never past the waypoint
  float carrotE = uE * carrot, carrotN = uN * carrot;    // carrot point on the line
  eastM  = carrotE - posE;                               // vector from us to the carrot
  northM = carrotN - posN;
}

// The waypoint to fly to. Past the last waypoint, hover over the launch point
// at the final leg's altitude.
inline Waypoint getMissionWaypoint(int index, double launchLat, double launchLon) {
  if (index < WAYPOINT_COUNT) return WAYPOINTS[index];
  float holdAlt = WAYPOINT_COUNT > 0 ? WAYPOINTS[WAYPOINT_COUNT - 1].altFt : 10.0f;
  return { launchLat, launchLon, holdAlt };
}
