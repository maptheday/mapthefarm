#pragma once

// ============================================================================
// PARKED -- on the ground, motors dead, waiting for the START switch.
// This is the safe resting state and the emergency-stop destination.
// ============================================================================

#include "IFlightPhase.hpp"
#include "../state/PhaseState.hpp"
#include "../state/FlightSettings.hpp"        // settings().safety.maxFlightTimeMs
#include "../services/Motors.hpp"           // Motors
#include "../services/MotorController.hpp"  // MotorController

class ParkedPhase : public IFlightPhase {
public:
  ParkedPhase(Motors* motors, MotorController* motorController) : motors_(motors), motorController_(motorController) {}

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
    motorController_->reset();
    // Disarm every tick so an e-stop never relies on an ESC-side timeout.
    // disarm() sends the 1000 us "stopped" pulse to every ESC.
    motors_->disarmAll();
  }

private:
  Motors*           motors_;
  MotorController*  motorController_;
};
