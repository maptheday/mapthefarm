#pragma once

// ============================================================================
// THE SERVICE SURFACE
// ----------------------------------------------------------------------------
// ELI5: these are the shared "tools on the workbench" that any phase is allowed
// to reach for. The self-contained services live in their own files (included
// below): motorController (PID+mixing), motors (the ESCs), and the NavMath
// helpers. This file just adds the few things wired into the agent host itself
// -- logging and the phase-switch function -- and re-exports the services so a
// phase only has to #include this one header.
// A phase never talks to the ESC pins or WiFi directly; it goes through these.
// ============================================================================

#include <Arduino.h>
#include "../models/FlightModel.hpp"
#include "../services/Log.hpp"              // logLine
#include "../services/MotorController.hpp"  // motorController
#include "../services/Motors.hpp"           // motors
#include "../services/Compass.hpp"          // compass
#include "../services/NavMath.hpp"          // gpsDistanceMeters, gpsBearing, ...

// Unrecoverable-bug halt: print once and freeze so a broken drone never flies.
#define PANIC(msg) do { Serial.println(F("[PANIC] " msg " — halting")); while(1) { delay(10); } } while(0)

// The service instances, defined once in the .ino (the agent host).
extern MotorController motorController;  // PID + motor mixing
extern Motors          motors;           // the 4 ESCs
extern Compass         compass;          // magnetometer + calibration

// Ask the drone to change phase (implemented by the phase machine,
// phases/PhaseMachine.hpp). Under WOKWI_SIM it's gated by the HIL harness;
// on real hardware it happens immediately.
void transitionTo(FlightPhase next, TransitionReason reason = REASON_NONE);

// Safety checks every flying phase runs each nav tick (max flight time,
// geofence, GPS loss). May itself trigger a transition to RTL or LANDING.
void checkCoreFailsafes(unsigned long armedAtMs, double launchLat, double launchLon);
