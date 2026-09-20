#pragma once

// ============================================================================
// THE PHASE MACHINE -- the one place phases are switched.
// transitionTo() is the whole state machine: it builds the entry context once
// (carrying arm-time + launch point forward from the previous flight phase),
// hands off to the new phase's onEnter(), and records the new phase.
//
// Transitions are immediate -- the same on a real drone and in the on-chip
// simulator (SIM). (The old laptop HIL "gate" that paused for a test harness to
// approve each transition is gone, along with the whole serial-injection rig.)
// ============================================================================

#include "../state/PhaseState.hpp"
#include "IFlightPhase.hpp"
#include "PhaseRegistry.hpp"
#include "PhaseSwitch.hpp"           // transitionTo() declaration (this file is its body)
#include "../services/Log.hpp"       // logLine
#include "../models/FlightModel.hpp"

inline void transitionTo(FlightPhase next, TransitionReason reason) {
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
      case PHASE_RTL_CLIMB:
        ctx.carriedArmedAtMs = shared.trip_rtlClimb.armedAtMs;
        ctx.carriedLaunchLat = shared.trip_rtlClimb.launchLat;
        ctx.carriedLaunchLon = shared.trip_rtlClimb.launchLon;
        break;
      case PHASE_RTL_RETURN:
        ctx.carriedArmedAtMs = shared.trip_rtlReturn.armedAtMs;
        ctx.carriedLaunchLat = shared.trip_rtlReturn.launchLat;
        ctx.carriedLaunchLon = shared.trip_rtlReturn.launchLon;
        break;
      default: break;
    }

    // Hand off to the phase we're entering. onEnter runs under this lock.
    phaseFor(next)->onEnter(ctx);

    shared.phase = next;
    shared.transitionReason = reason;
  });
}
