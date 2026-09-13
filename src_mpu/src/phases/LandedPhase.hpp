#pragma once

// ============================================================================
// LANDED -- touched down, motors cut. Like PARKED, but reached by finishing a
// flight rather than never leaving the ground. START can re-arm from here.
// ============================================================================

#include "IFlightPhase.hpp"
#include "../state/PhaseState.hpp"
#include "../state/FlightConfig.hpp"        // MAX_FLIGHT_TIME_MS
#include "../services/Motors.hpp"           // motors
#include "../services/MotorController.hpp"  // motorController

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
};
