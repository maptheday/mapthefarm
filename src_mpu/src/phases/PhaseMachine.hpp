#pragma once

// ============================================================================
// THE PHASE MACHINE -- the one place phases are switched.
// transitionTo() is the whole state machine: it builds the entry context once
// (carrying arm-time + launch point forward from the previous flight phase),
// hands off to the new phase's onEnter(), and records the new phase.
//
// Under WOKWI_SIM it also runs the HIL gate: instead of switching immediately,
// the first call records a "requested" transition and the test harness must
// approve it (ALLOW:) before hilGateBlocked() actually applies it on the next
// task tick. That makes phase changes observable + controllable from tests.
// On real hardware there is no gate -- transitions happen immediately.
// ============================================================================

#include "../state/PhaseState.hpp"
#include "IFlightPhase.hpp"
#include "PhaseRegistry.hpp"
#include "FlightRuntime.hpp"      // transitionTo() declaration, logLine, PANIC
#include "../models/FlightModel.hpp"
#ifdef WOKWI_SIM
#include "../state/HilState.hpp"  // sim GPS + HIL gate state
#endif

#ifdef WOKWI_SIM
// Returns true only once the harness has approved this exact transition.
inline bool hilGateAllows(FlightPhase next, TransitionReason reason) {
  if (!hilGatePending) {
    hilGatePending = true;
    hilGateApproved = false;
    hilGateNext = next;
    hilGateReason = reason;
    logLine(String("[HIL_GATE] request=") + phaseName(next) +
            " reason=" + reasonName(reason));
    return false;
  }
  return hilGateNext == next && hilGateApproved;
}
#endif

inline void transitionTo(FlightPhase next, TransitionReason reason) {
#ifdef WOKWI_SIM
  if (!hilGateAllows(next, reason)) return;
#endif
  withMutex([&]() {
    // Build the entry context ONCE here (the machine's job), including carrying
    // the arm time + launch point forward from the previous flight phase. Each
    // phase's onEnter() then only sets up its OWN state.
    EnterContext ctx;
    ctx.prevPhase         = shared.phase;
    ctx.now               = millis();
    ctx.currentAltFt      = shared.raw.baroAltitudeFt;
    ctx.currentHeadingDeg = shared.raw.compassHeadingDeg;
    ctx.currentLat        = shared.raw.gps.lat;
    ctx.currentLon        = shared.raw.gps.lon;
  #ifdef WOKWI_SIM
    ctx.currentLat        = simGpsLat;
    ctx.currentLon        = simGpsLon;
  #endif

    switch (ctx.prevPhase) {
      case PHASE_RAISE:
        ctx.carriedArmedAtMs = shared.trip_raise.armedAtMs;
        ctx.carriedLaunchLat = shared.trip_raise.launchLat;
        ctx.carriedLaunchLon = shared.trip_raise.launchLon;
        break;
      case PHASE_HOLD:
        ctx.carriedArmedAtMs = shared.trip_hold.armedAtMs;
        ctx.carriedLaunchLat = shared.trip_hold.launchLat;
        ctx.carriedLaunchLon = shared.trip_hold.launchLon;
        break;
      case PHASE_MISSION:
        ctx.carriedArmedAtMs = shared.trip_mission.armedAtMs;
        ctx.carriedLaunchLat = shared.trip_mission.launchLat;
        ctx.carriedLaunchLon = shared.trip_mission.launchLon;
        break;
      case PHASE_RTL:
        ctx.carriedArmedAtMs = shared.trip_rtl.armedAtMs;
        ctx.carriedLaunchLat = shared.trip_rtl.launchLat;
        ctx.carriedLaunchLon = shared.trip_rtl.launchLon;
        break;
      default: break;
    }

    // Hand off to the phase we're entering. onEnter runs under this lock.
    phaseFor(next)->onEnter(ctx);

    shared.phase = next;
    shared.transitionReason = reason;
  });
}

#ifdef WOKWI_SIM
// Called at the top of each task loop: if a gated transition has been approved,
// apply it now. Returns true while a transition is still pending (so the task
// skips its normal work until the harness lets the drone move on).
inline bool hilGateBlocked() {
  if (!hilGatePending) return false;
  if (hilGateApproved) {
    FlightPhase next = hilGateNext;
    TransitionReason reason = hilGateReason;
    transitionTo(next, reason);
    hilGatePending = false;
    hilGateApproved = false;
    hilGateReason = REASON_NONE;
  }
  return true;
}
#endif
