#pragma once

// ============================================================================
// FLIGHT CONSTANTS -- the few true internals that are NOT settings.
// Nobody should tune these in the field: changing them changes how the code
// itself works (loop timing). Everything a
// person might tune lives in data/flightsettings.json (see FlightSettings.hpp).
// ============================================================================

// The two control loops (see FlightController.hpp).
const unsigned long PHYSICS_LOOP_MS = 5;                       // 200 Hz: stability, motors
const float         PHYSICS_LOOP_HZ = 1000.0f / PHYSICS_LOOP_MS;
const unsigned long NAV_LOOP_MS     = 100;                     // 10 Hz: navigation, failsafes
