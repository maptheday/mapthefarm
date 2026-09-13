#pragma once

// ============================================================================
// The one unavoidable forward-declaration in the codebase.
//
// transitionTo() is fully implemented in phases/PhaseMachine.hpp. But there's a
// real dependency CYCLE: a phase CALLS transitionTo(), and transitionTo() calls
// back INTO the phases (phaseFor(next)->onEnter(...)). If a phase included the
// machine's body, and the machine includes every phase, they'd include each
// other forever.
//
// So a phase includes THIS tiny header to learn only transitionTo's shape (its
// prototype), enough to call it. The single real body is pasted into the build
// once, by the .ino including PhaseMachine.hpp. This is the C/C++ way to break a
// cycle -- everything else in the project keeps its declaration and definition
// together in one file.
// ============================================================================

#include "../models/FlightModel.hpp"  // FlightPhase, TransitionReason

void transitionTo(FlightPhase next, TransitionReason reason = REASON_NONE);
