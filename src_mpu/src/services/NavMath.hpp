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

// The waypoint to fly to. Past the last waypoint, hover over the launch point
// at the final leg's altitude.
inline Waypoint getMissionWaypoint(int index, double launchLat, double launchLon) {
  if (index < WAYPOINT_COUNT) return WAYPOINTS[index];
  float holdAlt = WAYPOINT_COUNT > 0 ? WAYPOINTS[WAYPOINT_COUNT - 1].altFt : 10.0f;
  return { launchLat, launchLon, holdAlt };
}
