#pragma once

// ============================================================================
// RC INPUT service -- what the pilot's radio switches MEAN.
//
// The radio itself is a plug (IRadio in FlightIo.hpp): the real CRSF receiver
// on the drone, or the sim's fake radio. Either way it hands over RadioFrames,
// and the flight controller's radio loop (FlightController.hpp) turns switch
// flips into fc::start(), fc::stop(), ... which land here:
//   STOP    level-triggered: any frame with STOP engaged -> emergency stop.
//   START   edge-triggered:  arm + take off (PARKED/LANDED), or start the
//                            mission (HOLD).
//   MANUAL  edge-triggered both ways: sticks on / back to auto-hover.
//   LAND    edge-triggered:  land where it is.
// These are the RULES behind each button (is it allowed right now? which phase
// next?). The buttons themselves are the public API: fc::start(), fc::land(), ...
// ============================================================================

#include <Arduino.h>
#include "../state/FlightSettings.hpp"        // settings().battery
#include "../state/PhaseState.hpp"          // shared, withMutex
#include "../services/Log.hpp"              // logLine
#include "../phases/PhaseSwitch.hpp"        // transitionTo

// STOP switch: cut motors unless already on the ground.
inline void crsfHandleStop() {
  FlightPhase phase;
  withMutex([&]() { phase = shared.phase; });
  if (phase != PHASE_PARKED && phase != PHASE_LANDED) {
    logLine("[CRSF] STOP switch -- emergency stop.");
    transitionTo(PHASE_PARKED, REASON_EMERGENCY_STOP);
  }
}

// START switch: arm+takeoff from the ground, or start the mission from HOLD.
inline void crsfHandleStart() {
  FlightPhase phase;
  bool hasFix;
  RawBattery batt;
  withMutex([&]() { phase = shared.phase; hasFix = shared.raw.gps.fix; batt = shared.raw.battery; });

  // LANDED re-arms like PARKED: motors are already confirmed off in both, so a
  // landed drone isn't "busy" -- it can fly again without a separate reset.
  if (phase == PHASE_PARKED || phase == PHASE_LANDED) {
    if (!hasFix) {
      logLine("[CRSF] START ignored -- no GPS fix.");
    } else if (batt.state != BATTERY_OK) {
      logLine("[CRSF] START ignored -- battery low. Swap in a charged pack.");
    } else if (batt.present && batt.cellVolts < settings().battery.cellTakeoffMinV) {
      // Motors are off here, so this is the resting voltage.
      logLine("[CRSF] START ignored -- battery too low to take off (" +
              String(batt.cellVolts, 2) + " V/cell). Charge it first.");
    } else {
      logLine("[CRSF] START switch -- arming and taking off.");
      transitionTo(PHASE_RAISE, REASON_OPERATOR_START);
    }
  } else if (phase == PHASE_HOLD) {
    if (batt.state != BATTERY_OK) {
      logLine("[CRSF] START ignored -- battery low, not starting the mission.");
      return;
    }
    logLine("[CRSF] START switch -- starting waypoint mission.");
    transitionTo(PHASE_MISSION, REASON_OPERATOR_START);
  } else {
    logLine("[CRSF] START ignored -- already flying/busy (phase: "
            + String(phaseName(phase)) + ")");
  }
}

// LAND switch flipped UP: land right where it is, from anything that's flying.
// The gentle way down (STOP just cuts the motors, which drops the drone).
inline void crsfHandleLand() {
  FlightPhase phase;
  withMutex([&]() { phase = shared.phase; });
  bool airborne = phase == PHASE_RAISE || phase == PHASE_HOLD || phase == PHASE_MISSION ||
                  phase == PHASE_MANUAL || phase == PHASE_HOVER_SETTLE ||
                  phase == PHASE_RTL_CLIMB || phase == PHASE_RTL_RETURN || phase == PHASE_RTL_SETTLE;
  if (airborne) {
    logLine("[CRSF] LAND switch -- landing here.");
    transitionTo(PHASE_LANDING, REASON_OPERATOR_LAND);
  } else {
    logLine("[CRSF] LAND ignored (phase: " + String(phaseName(phase)) + ")");
  }
}

// MANUAL switch flipped ON: pilot takes the sticks. Allowed from the ground
// (bench/hand testing) or from a stable auto-hover (take over mid-flight).
inline void crsfHandleManualOn() {
  FlightPhase phase;
  withMutex([&]() { phase = shared.phase; });
  if (phase == PHASE_PARKED || phase == PHASE_LANDED || phase == PHASE_HOLD) {
    logLine("[CRSF] MANUAL switch -- pilot has the sticks.");
    transitionTo(PHASE_MANUAL, REASON_MANUAL_ON);
  } else {
    logLine("[CRSF] MANUAL ignored (phase: " + String(phaseName(phase)) + ")");
  }
}

// MANUAL switch flipped OFF: hand back to auto-hover (HOLD). STOP is still the
// hard kill; this is the "let go of the sticks safely" path.
inline void crsfHandleManualOff() {
  FlightPhase phase;
  withMutex([&]() { phase = shared.phase; });
  if (phase == PHASE_MANUAL) {
    logLine("[CRSF] MANUAL switch off -- handing back to auto-hover.");
    transitionTo(PHASE_HOLD, REASON_MANUAL_OFF);
  }
}
