#pragma once

// ============================================================================
// CALIBRATE -- ground maintenance, motors OFF. The operator slowly rotates the
// drone through every orientation while we collect the compass min/max, then
// we compute + save the calibration and drop back to PARKED.
//
// This is the template for future calibrations (accelerometer, gyro, ESC): a
// self-contained ground phase that drives one plug and returns to PARKED.
// ============================================================================

#include "IFlightPhase.hpp"
#include "../state/PhaseState.hpp"
#include "../state/FlightSettings.hpp"        // settings().calibration.compassDurationMs
#include "../FlightIo.hpp"                   // flightIo().compass
#include "../services/Motors.hpp"           // motors
#include "../services/Log.hpp"              // logLine
#include "PhaseSwitch.hpp"                  // transitionTo

class CalibratePhase : public IFlightPhase {
public:
  FlightPhase id() const override { return PHASE_CALIBRATE; }

  void onEnter(const EnterContext& ctx) override {
    shared.trip_calibrate.enteredAtMs   = ctx.now;
    shared.dashboard_calibrate.progressPct = 0.0f;
    flightIo().compass->startCalibration();
    logLine("[COMPASS] Calibration starting — rotate drone slowly through all axes now...");
  }

  void navTick(float /*navDt*/) override {
    flightIo().compass->sampleCalibration();

    unsigned long enteredAt;
    withMutex([&]() { enteredAt = shared.trip_calibrate.enteredAtMs; });
    unsigned long elapsed = millis() - enteredAt;

    withMutex([&]() {
      shared.dashboard_calibrate.progressPct =
        min(100.0f, 100.0f * (float)elapsed / (float)settings().calibration.compassDurationMs);
    });

    if (elapsed >= settings().calibration.compassDurationMs) {
      flightIo().compass->finishCalibration();
      logLine("[COMPASS] Calibration saved.");
      transitionTo(PHASE_PARKED, REASON_CALIBRATION_COMPLETE);
    }
  }

  void physicsTick(float /*dt*/) override {
    // Motors must stay off -- the operator is handling the drone.
    motors.disarmAll();
  }
};
