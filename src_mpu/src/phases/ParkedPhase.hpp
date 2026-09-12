#pragma once

// ============================================================================
// PARKED -- on the ground, motors dead, waiting for the START switch.
// This is the safe resting state and the emergency-stop destination.
// ============================================================================

#include "IFlightPhase.hpp"
#include "../state/PhaseState.hpp"
#include "FlightRuntime.hpp"

class ParkedPhase : public IFlightPhase {
public:
  FlightPhase id() const override { return PHASE_PARKED; }

  // Nothing to set up: parking just means "sit still with motors off".

  void navTick(float /*dt*/) override {
    // Keep the dashboard showing live attitude so the web page isn't frozen.
    withMutex([&]() {
      shared.dashboard_parked.altitudeFt = shared.raw.baroAltitudeFt;
      shared.dashboard_parked.roll       = shared.raw.imu.gyroX;
      shared.dashboard_parked.pitch      = shared.raw.imu.gyroY;
      shared.dashboard_parked.yaw        = shared.raw.imu.gyroZ;
    });
  }

  void physicsTick(float /*dt*/) override {
    motorController.reset();
    // Disarm every tick so an e-stop never relies on an ESC-side timeout.
    // DShot has no PWM "min throttle" -- disarm() sends the real stop command.
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
