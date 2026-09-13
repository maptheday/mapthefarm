#pragma once

// ============================================================================
// PARKED -- on the ground, motors dead, waiting for the START switch.
// This is the safe resting state and the emergency-stop destination.
// ============================================================================

#include "IFlightPhase.hpp"
#include "../state/PhaseState.hpp"
#include "../state/FlightConfig.hpp"        // MAX_FLIGHT_TIME_MS
#include "../services/Motors.hpp"           // motors
#include "../services/MotorController.hpp"  // motorController

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
};
