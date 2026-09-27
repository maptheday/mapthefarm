#pragma once

// ============================================================================
// FAILSAFES service -- the safety net every flying phase runs each nav tick.
// Three independent checks, in priority order:
//   1. Max flight time  -> force RTL
//   2. Geofence breach  -> force RTL
//   3. GPS fix lost      -> abort straight to LANDING (can't RTL without GPS)
// It reads the shared state and, when a limit is hit, drives transitionTo().
//
// Plus a separate radio-link check, checkRadioFailsafe() (bottom of this file):
//   radio silent for RC_LOSS_TIMEOUT_MS -> RTL (or LAND if low / no GPS).
// ============================================================================

#include "../state/PhaseState.hpp"
#include "../state/FlightConfig.hpp"   // GEOFENCE_RADIUS_M, GPS_LOSS_ABORT_MS
#include "../models/FlightModel.hpp"
#include "NavMath.hpp"                 // gpsDistanceMeters
#include "Log.hpp"                     // logLine
#include "../phases/PhaseSwitch.hpp"   // transitionTo

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

  if (fix && launchLat != 0.0) {
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

  bool alreadyComingHome = phase == PHASE_RTL_CLIMB || phase == PHASE_RTL_RETURN ||
                           phase == PHASE_RTL_SETTLE || phase == PHASE_LANDING ||
                           phase == PHASE_LANDED;
  if (tripRTL && !alreadyComingHome) {
    transitionTo(PHASE_RTL_CLIMB, rtlReason);  // RTL starts at the climb step
  }
}


// ----------------------------------------------------------------------------
// RADIO LINK LOSS -- the pilot can't reach the drone any more (out of range,
// radio battery died). ELRS receivers go silent when the link drops, so "no
// radio frame for RC_LOSS_TIMEOUT_MS" means the STOP and MANUAL switches no
// longer work. Called from every phase where the pilot could be relying on
// the radio: RAISE, HOLD, MISSION, MANUAL. Returns true if it changed phase
// (the caller should stop its tick there).
//
//   high up, with GPS  -> RTL_CLIMB (come home, then land)
//   low (< RC_LOSS_LAND_BELOW_FT), no GPS fix, or home unknown -> LANDING here
//
// Only armed once a radio has actually been heard (rcLastFrameMs > 0), so it
// never fires in the sim or before a radio is connected.
// ----------------------------------------------------------------------------
inline bool checkRadioFailsafe() {
  unsigned long lastFrame;
  bool          fix;
  float         altFt;
  FlightPhase   phase;
  double        manualLaunchLat;
  withMutex([&]() {
    lastFrame       = shared.rcLastFrameMs;
    fix             = shared.raw.gps.fix;
    altFt           = shared.raw.baroAltitudeFt;
    phase           = shared.phase;
    manualLaunchLat = shared.trip_manual.launchLat;
  });

  if (lastFrame == 0) return false;                               // never heard a radio
  if (millis() - lastFrame < RC_LOSS_TIMEOUT_MS) return false;    // link is fine

  bool pilotReliant = phase == PHASE_RAISE || phase == PHASE_HOLD ||
                      phase == PHASE_MISSION || phase == PHASE_MANUAL;
  if (!pilotReliant) return false;   // on the ground, or already coming home/landing

  // RTL needs to know where home is. MANUAL entered straight from the ground
  // (PARKED) never recorded a launch point, so it can only land.
  bool homeKnown = !(phase == PHASE_MANUAL && manualLaunchLat == 0.0);

  if (!fix || altFt < RC_LOSS_LAND_BELOW_FT || !homeKnown) {
    logLine("[SAFETY] Radio link lost — landing here.");
    transitionTo(PHASE_LANDING, REASON_RC_LOST);
  } else {
    logLine("[SAFETY] Radio link lost — returning to launch.");
    transitionTo(PHASE_RTL_CLIMB, REASON_RC_LOST);
  }
  return true;
}
