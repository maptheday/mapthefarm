#pragma once

// ============================================================================
// LANDED -- touched down, motors cut. Like PARKED, but reached by finishing a
// flight rather than never leaving the ground. START can re-arm from here.
// ============================================================================

#include "IFlightPhase.hpp"
#include "../state/PhaseState.hpp"
#include "../state/FlightSettings.hpp"        // settings().safety.maxFlightTimeMs
#include "../services/Motors.hpp"           // Motors
#include "../services/MotorController.hpp"  // MotorController

class LandedPhase : public IFlightPhase {
public:
  LandedPhase(Motors* motors, MotorController* motorController) : motors_(motors), motorController_(motorController) {}

  FlightPhase id() const override { return PHASE_LANDED; }

  void navTick(float /*dt*/) override {
    withMutex([&]() { shared.dashboard_landed.altitudeFt = shared.raw.baroAltitudeFt; });
  }

  void physicsTick(float /*dt*/) override {
    motorController_->reset();
    // Same reasoning as PARKED: disarm explicitly, don't trust last ESC state.
    motors_->disarmAll();
  }

private:
  Motors*           motors_;
  MotorController*  motorController_;
};
