#pragma once

// ============================================================================
// THE REGISTRY -- the board every phase "tool" plugs into.
// ----------------------------------------------------------------------------
// This is the ONLY place that lists all phases. The flight loops just call
// phaseFor(currentPhase)->whatever(), so adding or renaming a phase means
// editing exactly one file (plus its own).
// ============================================================================

#include "IFlightPhase.hpp"
#include "ParkedPhase.hpp"
#include "RaisePhase.hpp"
#include "HoldPhase.hpp"
#include "MissionPhase.hpp"
#include "RtlClimbPhase.hpp"
#include "RtlReturnPhase.hpp"
#include "RtlSettlePhase.hpp"
#include "HoverSettlePhase.hpp"
#include "LandingPhase.hpp"
#include "LandedPhase.hpp"
#include "CalibratePhase.hpp"
#include "ManualPhase.hpp"

// One slot per FlightPhase (PHASE_MANUAL is the last one in the enum -- if you
// add a phase after it, size the table by the new last one).
inline IFlightPhase** phaseTable() { static IFlightPhase* table[PHASE_MANUAL + 1] = {}; return table; }

// Make one of each phase, handing each exactly what it needs (constructor
// injection, like C#). Called once by fc::begin(). (One instance each is
// enough: the phases keep their state in the shared notebook, not in
// themselves.)
inline void buildPhases(Motors* motors, MotorController* motorController,
                        Failsafes* failsafes, ICompass* compass) {
  IFlightPhase** table = phaseTable();
  table[PHASE_PARKED]       = new ParkedPhase(motors, motorController);
  table[PHASE_RAISE]        = new RaisePhase(motors, motorController, failsafes);
  table[PHASE_HOLD]         = new HoldPhase(motors, motorController, failsafes);
  table[PHASE_MISSION]      = new MissionPhase(motors, motorController, failsafes);
  table[PHASE_RTL_CLIMB]    = new RtlClimbPhase(motors, motorController, failsafes);
  table[PHASE_RTL_RETURN]   = new RtlReturnPhase(motors, motorController, failsafes);
  table[PHASE_RTL_SETTLE]   = new RtlSettlePhase(motors, motorController, failsafes);
  table[PHASE_HOVER_SETTLE] = new HoverSettlePhase(motors, motorController, failsafes);
  table[PHASE_LANDING]      = new LandingPhase(motors, motorController);
  table[PHASE_LANDED]       = new LandedPhase(motors, motorController);
  table[PHASE_CALIBRATE]    = new CalibratePhase(motors, compass);
  table[PHASE_MANUAL]       = new ManualPhase(motors, motorController, failsafes);
}

// The object that runs a phase.
inline IFlightPhase* phaseFor(FlightPhase phase) { return phaseTable()[phase]; }
