#pragma once

// ============================================================================
// THE PLUGIN SOCKET  (think: a C# interface)
// ----------------------------------------------------------------------------
// A flight phase is a "tool" you plug into the drone. The ESP (agent) keeps a
// table of these and, on every loop, calls the one method it needs on the
// CURRENT phase -- it never cares which concrete phase is plugged in.
//
// This interface has NO shared behaviour: every phase implements its own
// methods from scratch. That is deliberate -- a bug in one phase cannot leak
// into another, and you can open a single phase file and understand it alone.
//
// Four things a phase can do:
//   onEnter()       once, the moment the drone switches into this phase
//   navTick()       ~10 Hz  -> navigation decisions, failsafes, transitions
//   physicsTick()   ~200 Hz -> read targets, run PID, drive the motors
//   writeTelemetry()          -> fill the JSON the web page / HIL reads
// Any method a phase doesn't need it simply leaves as the empty default.
// ============================================================================

#include <ArduinoJson.h>
#include "../models/FlightModel.hpp"

// Everything a phase needs to set itself up on entry. transitionTo() fills this
// in ONCE (including carrying the arm-time and launch point forward from the
// previous flight phase), so each phase's onEnter only initialises its OWN
// state and never has to know what came before.
struct EnterContext {
  FlightPhase   prevPhase         = PHASE_PARKED;
  unsigned long now               = 0;
  float         currentAltFt      = 0.0f;
  float         currentHeadingDeg = 0.0f;
  double        currentLat        = 0.0;
  double        currentLon        = 0.0;
  unsigned long carriedArmedAtMs  = 0;   // arm time from the previous flight phase
  double        carriedLaunchLat  = 0.0; // launch point from the previous flight phase
  double        carriedLaunchLon  = 0.0;
};

class IFlightPhase {
public:
  virtual ~IFlightPhase() {}

  // Which phase this tool implements (must match its slot in the registry).
  virtual FlightPhase id() const = 0;

  // Runs while transitionTo() HOLDS the state lock -- so touch `shared`
  // directly here and do NOT call withMutex() (that would try to grab the pen
  // the caller is already holding). Set up this phase's own state only.
  virtual void onEnter(const EnterContext& ctx) {}

  // Called without the lock held -- use withMutex() when you touch `shared`.
  virtual void navTick(float dt) {}
  virtual void physicsTick(float dt) {}

  // Fill in the phase-specific telemetry fields. The shared fields
  // (flightPhase / flightEnabled / altFt) are already set by the caller.
  virtual void writeTelemetry(JsonDocument& doc) {}
};
