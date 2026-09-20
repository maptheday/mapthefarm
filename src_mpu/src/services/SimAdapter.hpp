#pragma once

// ============================================================================
// SIM ADAPTER -- the tiny USB command interface for the on-chip simulator.
// Sim builds only. The flight itself is fully autonomous (see OnboardSim); this
// only listens for two text commands over USB:
//   SCENARIO:<name>  pick the test scenario at boot (full / geofence / gpsloss /
//                    timeout / manual / stab) -- default is "full"
//   DUMPLOG          stream the recorded flight log back (between clear markers)
//   PING:            liveness check -- confirms the firmware is up
// (The old laptop HIL protocol -- sensor injection, the approval gate, per-tick
// stick faking -- is gone; the physics and the flight all run on the ESP now.)
// ============================================================================

#include <Arduino.h>

#ifdef SIM

#include "../services/Log.hpp"     // logLine
#include "OnboardSim.hpp"          // onboardSimDumpLog

inline void parseSimInput() {
  static String buf = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      if (buf.startsWith("SCENARIO:")) {
        onboardSimSetScenario(buf.substring(9));   // pick the test scenario (before takeoff)
      }
      else if (buf.startsWith("DUMPLOG")) {
        onboardSimDumpLog();       // stream the on-chip flight log over USB
      }
      else if (buf.startsWith("PING:")) {
        logLine("[SIM] Ready.");
      }
      buf = "";
    } else if (c != '\r') {
      buf += c;
    }
  }
}

#endif // SIM
