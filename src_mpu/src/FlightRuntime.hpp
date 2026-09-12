#pragma once

// ============================================================================
// THE SERVICE SURFACE
// ----------------------------------------------------------------------------
// ELI5: these are the shared "tools on the workbench" that any phase is allowed
// to reach for -- the PID/motor-mixer service, the GPS math, the motor outputs,
// logging, and the one function that switches phases. They are DECLARED here so
// a phase file can call them, and DEFINED once in the .ino (the agent host).
// A phase never talks to the ESC pins or WiFi directly; it goes through these.
// ============================================================================

#include <Arduino.h>
#include "FlightModel.hpp"
#include "FlightConfig.hpp"
#include "ControlTypes.hpp"
#include "MotorController.hpp"

// Unrecoverable-bug halt: print once and freeze so a broken drone never flies.
#define PANIC(msg) do { Serial.println(F("[PANIC] " msg " — halting")); while(1) { delay(10); } } while(0)

// The one PID/mixing service, shared by every flying phase.
extern MotorController motorController;

// Thread-safe serial logging (used for HIL log lines too -- keep messages exact).
void logLine(const String& msg);

// Ask the drone to change phase. Under WOKWI_SIM this is gated by the HIL
// harness; on real hardware it happens immediately. Defined in the .ino.
void transitionTo(FlightPhase next, TransitionReason reason = REASON_NONE);

// Push a computed 4-motor mix to the ESCs / cut all motors.
void writeMotorMix(const MotorMix& mix);
void disarmAllMotors();

// Safety checks every flying phase runs each nav tick (max flight time,
// geofence, GPS loss). May itself trigger a transition to RTL or LANDING.
void checkCoreFailsafes(unsigned long armedAtMs, double launchLat, double launchLon);

// GPS / navigation math.
float gpsDistanceMeters(double lat1, double lon1, double lat2, double lon2);
float gpsBearing(double lat1, double lon1, double lat2, double lon2);
void  bearingToNorthEast(float distM, float bearingDeg, float& northM, float& eastM);
Waypoint getMissionWaypoint(int index, double launchLat, double launchLon);
