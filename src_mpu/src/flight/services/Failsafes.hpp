#pragma once

// ============================================================================
// FAILSAFES service -- the safety net every flying phase runs each nav tick.
// Three independent checks, in priority order:
//   1. Max flight time  -> LAND where it is (on a big field, flying home could
//                          cost more battery than is left)
//   2. Geofence breach  -> force RTL
//   3. GPS fix lost      -> abort straight to LANDING (can't RTL without GPS)
// It reads the shared state and, when a limit is hit, drives transitionTo().
//
// Plus two separate checks at the bottom of this file:
//   checkRadioFailsafe()   radio silent for settings().safety.radioLossTimeoutMs -> RTL (or LAND if low / no GPS)
//   checkBatteryFailsafe() battery WARNING or CRITICAL -> LAND where it is
// ============================================================================

#include "../state/PhaseState.hpp"
#include "../state/FlightSettings.hpp"   // settings().safety.geofenceRadiusM, settings().safety.gpsLossAbortMs
#include "../models/FlightModel.hpp"
#include "NavMath.hpp"                 // gpsDistanceMeters
#include "Log.hpp"                     // logLine
#include "../phases/PhaseSwitch.hpp"   // transitionTo

inline void checkCoreFailsafes(unsigned long armedAtMs, double launchLat, double launchLon) {
  bool tripRTL = false;
  TransitionReason rtlReason = REASON_NONE;

  FlightPhase phase;
  withMutex([&]() { phase = shared.phase; });

  // 1. Max flight time -> land right here. The timer is a fuel gauge, and once
  //    it runs out there may not be enough left to fly home across a big field.
  if (armedAtMs > 0 && (millis() - armedAtMs) >= settings().safety.maxFlightTimeMs &&
      phase != PHASE_LANDING && phase != PHASE_LANDED) {
    logLine("[SAFETY] Max flight time reached — landing here.");
    transitionTo(PHASE_LANDING, REASON_MAX_FLIGHT_TIME);
    return;
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
    if (gpsDistanceMeters(lat, lon, launchLat, launchLon) > settings().safety.geofenceRadiusM) {
      logLine("[SAFETY] Geofence exceeded — forcing RTL.");
      tripRTL = true;
      if (rtlReason == REASON_NONE) rtlReason = REASON_GEOFENCE;
    }
  }

  // 3. GPS loss
  if (!fix && lastFix > 0 && (millis() - lastFix) >= settings().safety.gpsLossAbortMs) {
    logLine("[SAFETY] GPS fix lost — aborting directly to LANDING.");
    transitionTo(PHASE_LANDING, REASON_GPS_LOSS); // can't RTL without GPS
    return;
  }

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
// radio frame for settings().safety.radioLossTimeoutMs" means the STOP and MANUAL switches no
// longer work. Called from every phase where the pilot could be relying on
// the radio: RAISE, HOLD, MISSION, MANUAL. Returns true if it changed phase
// (the caller should stop its tick there).
//
//   high up, with GPS  -> RTL_CLIMB (come home, then land)
//   low (< settings().safety.radioLossLandBelowFt), no GPS fix, or home unknown -> LANDING here
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
  if (millis() - lastFrame < settings().safety.radioLossTimeoutMs) return false;    // link is fine

  bool pilotReliant = phase == PHASE_RAISE || phase == PHASE_HOLD ||
                      phase == PHASE_MISSION || phase == PHASE_MANUAL;
  if (!pilotReliant) return false;   // on the ground, or already coming home/landing

  // RTL needs to know where home is. MANUAL entered straight from the ground
  // (PARKED) never recorded a launch point, so it can only land.
  bool homeKnown = !(phase == PHASE_MANUAL && manualLaunchLat == 0.0);

  if (!fix || altFt < settings().safety.radioLossLandBelowFt || !homeKnown) {
    logLine("[SAFETY] Radio link lost — landing here.");
    transitionTo(PHASE_LANDING, REASON_RC_LOST);
  } else {
    logLine("[SAFETY] Radio link lost — returning to launch.");
    transitionTo(PHASE_RTL_CLIMB, REASON_RC_LOST);
  }
  return true;
}


// ----------------------------------------------------------------------------
// BATTERY -- the Battery service (Battery.hpp) watches three gauges: estimated
// mAh used, filtered voltage, and (here, via checkCoreFailsafes) flight time.
// As soon as it reports WARNING or CRITICAL, LAND WHERE WE ARE. Don't try to
// fly home: on a big field, the trip back could cost more than is left.
//
// Called from every phase that's in the air: RAISE, HOLD, MISSION, MANUAL,
// HOVER_SETTLE and the three RTL phases. Returns true if it changed phase.
// ----------------------------------------------------------------------------
inline bool checkBatteryFailsafe() {
  uint8_t     state;
  FlightPhase phase;
  withMutex([&]() {
    state = shared.raw.battery.state;
    phase = shared.phase;
  });

  if (state == BATTERY_OK) return false;

  bool airborne = phase == PHASE_RAISE || phase == PHASE_HOLD || phase == PHASE_MISSION ||
                  phase == PHASE_MANUAL || phase == PHASE_HOVER_SETTLE ||
                  phase == PHASE_RTL_CLIMB || phase == PHASE_RTL_RETURN || phase == PHASE_RTL_SETTLE;
  if (!airborne) return false;   // on the ground, or already landing

  if (state == BATTERY_CRITICAL) {
    logLine("[SAFETY] Battery critical — landing here.");
    transitionTo(PHASE_LANDING, REASON_BATTERY_CRITICAL);
  } else {
    logLine("[SAFETY] Battery low — landing here.");
    transitionTo(PHASE_LANDING, REASON_BATTERY_LOW);
  }
  return true;
}
