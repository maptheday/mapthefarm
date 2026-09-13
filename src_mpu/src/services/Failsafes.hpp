#pragma once

// ============================================================================
// FAILSAFES service -- the safety net every flying phase runs each nav tick.
// Three independent checks, in priority order:
//   1. Max flight time  -> force RTL
//   2. Geofence breach  -> force RTL
//   3. GPS fix lost      -> abort straight to LANDING (can't RTL without GPS)
// It reads the shared state and, when a limit is hit, drives transitionTo().
// ============================================================================

#include "../state/PhaseState.hpp"
#include "../state/FlightConfig.hpp"   // GEOFENCE_RADIUS_M, GPS_LOSS_ABORT_MS
#include "../models/FlightModel.hpp"
#include "NavMath.hpp"                 // gpsDistanceMeters
#include "Log.hpp"                     // logLine
#include "../phases/PhaseSwitch.hpp"   // transitionTo
#ifdef WOKWI_SIM
#include "../state/HilState.hpp"       // sim GPS
#endif

inline void checkCoreFailsafes(unsigned long armedAtMs, double launchLat, double launchLon) {
  bool tripRTL = false;
  TransitionReason rtlReason = REASON_NONE;

  // 1. Max flight time
  if (armedAtMs > 0 && (millis() - armedAtMs) >= MAX_FLIGHT_TIME_MS) {
    logLine("[SAFETY] Max flight time reached — forcing RTL.");
    tripRTL = true;
    rtlReason = REASON_MAX_FLIGHT_TIME;
  }

  // 2. Geofence
  bool fix;
  double lat;
  double lon;
  unsigned long lastFix;
  withMutex([&]() {
    fix     = shared.raw.gps.fix;
    lat     = shared.raw.gps.lat;
    lon     = shared.raw.gps.lon;
    lastFix = shared.raw.gps.lastFixMs;
  });
#ifdef WOKWI_SIM
  fix      = simGpsFix;
  lat      = simGpsLat;
  lon      = simGpsLon;
  lastFix  = fix ? millis() : lastFix;
#endif

#ifdef WOKWI_SIM
  if (launchLat != 0.0) {
#else
  if (fix && launchLat != 0.0) {
#endif
    if (gpsDistanceMeters(lat, lon, launchLat, launchLon) > GEOFENCE_RADIUS_M) {
      logLine("[SAFETY] Geofence exceeded — forcing RTL.");
      tripRTL = true;
      if (rtlReason == REASON_NONE) rtlReason = REASON_GEOFENCE;
    }
  }

  // 3. GPS loss
  if (!fix && lastFix > 0 && (millis() - lastFix) >= GPS_LOSS_ABORT_MS) {
    logLine("[SAFETY] GPS fix lost — aborting directly to LANDING.");
    transitionTo(PHASE_LANDING, REASON_GPS_LOSS); // can't RTL without GPS
    return;
  }

  FlightPhase phase;
  withMutex([&]() { phase = shared.phase; });

  if (tripRTL && phase != PHASE_RTL && phase != PHASE_LANDING && phase != PHASE_LANDED) {
    transitionTo(PHASE_RTL, rtlReason);
  }
}
