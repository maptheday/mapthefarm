#pragma once

// ============================================================================
// THE REGISTRY -- the board every phase "tool" plugs into.
// ----------------------------------------------------------------------------
// This is the ONLY place that lists all phases. The two RTOS loops and the
// telemetry formatter just call phaseFor(currentPhase)->whatever(), so adding
// or renaming a phase means editing exactly one file (plus its own).
// ============================================================================

#include "IFlightPhase.hpp"
#include "ParkedPhase.hpp"
#include "RaisePhase.hpp"
#include "HoldPhase.hpp"
#include "MissionPhase.hpp"
#include "RtlPhase.hpp"
#include "HoverSettlePhase.hpp"
#include "LandingPhase.hpp"
#include "LandedPhase.hpp"
#include "CalibratePhase.hpp"
#include "ManualPhase.hpp"

// One instance of each phase (they hold no state of their own -- all state
// lives in the shared repository -- so a single shared instance is fine).
inline IFlightPhase* phaseFor(FlightPhase phase) {
  static ParkedPhase      parked;
  static RaisePhase       raise;
  static HoldPhase        hold;
  static MissionPhase     mission;
  static RtlPhase         rtl;
  static HoverSettlePhase hoverSettle;
  static LandingPhase     landing;
  static LandedPhase      landed;
  static CalibratePhase   calibrate;
  static ManualPhase      manual;

  // Table order MUST match the FlightPhase enum order in FlightModel.hpp.
  static IFlightPhase* table[] = {
    &parked,       // PHASE_PARKED
    &raise,        // PHASE_RAISE
    &hold,         // PHASE_HOLD
    &mission,      // PHASE_MISSION
    &rtl,          // PHASE_RTL
    &hoverSettle,  // PHASE_HOVER_SETTLE
    &landing,      // PHASE_LANDING
    &landed,       // PHASE_LANDED
    &calibrate,    // PHASE_CALIBRATE
    &manual,       // PHASE_MANUAL
  };
  return table[phase];
}
