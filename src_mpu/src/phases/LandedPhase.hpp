#pragma once

// ============================================================================
// LANDED -- touched down, motors cut. Like PARKED, but reached by finishing a
// flight rather than never leaving the ground. START can re-arm from here.
// ============================================================================

#include "IFlightPhase.hpp"
#include "../state/PhaseState.hpp"
#include "FlightRuntime.hpp"

class LandedPhase : public IFlightPhase {
public:
  FlightPhase id() const override { return PHASE_LANDED; }

  void navTick(float /*dt*/) override {
    withMutex([&]() { shared.dashboard_landed.altitudeFt = shared.raw.baroAltitudeFt; });
  }

  void physicsTick(float /*dt*/) override {
    motorController.reset();
    // Same reasoning as PARKED: disarm explicitly, don't trust last ESC state.
    motors.disarmAll();
  }

  void writeTelemetry(JsonDocument& doc) override {
    withMutex([&]() {
      doc["targetFt"]           = 0.0f;
      doc["m1"]                 = 0;
      doc["m2"]                 = 0;
      doc["m3"]                 = 0;
      doc["m4"]                 = 0;
      doc["baseThrottle"]       = 0;
      doc["rollCorrection"]     = 0;
      doc["pitchCorrection"]    = 0;
      doc["gpsFix"]             = shared.raw.gps.fix;
      doc["gpsLat"]             = shared.raw.gps.lat;
      doc["gpsLon"]             = shared.raw.gps.lon;
      doc["gpsSats"]            = shared.raw.gps.sats;
      doc["compassHeading"]     = shared.raw.compassHeadingDeg;
      doc["navActive"]          = false;
      doc["navWaypoint"]        = 0;
      doc["navWaypointCount"]   = 0;
      doc["navDistM"]           = 0.0;
      doc["navBearing"]         = 0.0;
      doc["flightSecRemaining"] = (long)(MAX_FLIGHT_TIME_MS / 1000);
    });
  }
};
