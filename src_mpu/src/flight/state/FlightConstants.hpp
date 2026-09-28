#pragma once

// ============================================================================
// FLIGHT CONSTANTS -- the few true internals that are NOT settings.
// Nobody should tune these in the field: changing them changes how the code
// itself works (loop timing, a radio protocol's fixed numbers). Everything a
// person might tune lives in data/flightsettings.json (see FlightSettings.hpp).
// ============================================================================

// The two control loops (see FlightController.hpp).
const unsigned long PHYSICS_LOOP_MS = 5;                       // 200 Hz: stability, motors
const float         PHYSICS_LOOP_HZ = 1000.0f / PHYSICS_LOOP_MS;
const unsigned long NAV_LOOP_MS     = 100;                     // 10 Hz: navigation, failsafes

// CRSF radio protocol (fixed by the protocol, not by us).
const unsigned long CRSF_BAUD    = 420000;
const int           CRSF_RAW_MIN = 172;    // stick fully one way
const int           CRSF_RAW_MID = 992;    // centered
const int           CRSF_RAW_MAX = 1811;   // fully the other way
