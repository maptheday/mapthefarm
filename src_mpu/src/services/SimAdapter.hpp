#pragma once

// ============================================================================
// SIM ADAPTER -- the hardware-in-the-loop (HIL) test interface. Sim builds only.
// The Python test runner drives the drone entirely over the USB serial line
// with tiny text commands; this file parses them and speaks the reply protocol:
//   inputs:  HDG:/LAT:/LON:/FIX:/ALT:  inject fake sensor readings
//            CRSFSTART:1 / CRSFSTOP:1   fake the pilot's radio switches
//            MISSION:                   shortcut to start a mission
//            ALLOW:<PHASE>              approve a gated transition
//            RESET:                     wipe state between scenarios
//            PING:                      liveness check
//   queries: STATUS? / MOTOR?          machine-readable state read-back
// Kept out of the real firmware path -- none of this compiles on hardware.
// ============================================================================

#include <Arduino.h>

#ifdef WOKWI_SIM

#include "../state/PhaseState.hpp"     // shared, withMutex, RTLState, Dashboard_RTL
#include "../state/HilState.hpp"       // sim inputs + HIL gate state
#include "../services/Log.hpp"         // logLine
#include "../phases/PhaseSwitch.hpp"   // transitionTo
#include "RcInput.hpp"                 // crsfHandleStart / crsfHandleStop

inline void resetSimState() {
  // HIL gate + injected sensor inputs (the sim-only globals).
  simGpsFix       = false;
  hilGatePending  = false;
  hilGateApproved = false;
  hilGateNext     = PHASE_PARKED;
  hilGateReason   = REASON_NONE;

  // Reset the WHOLE shared state to its struct defaults in one shot. Doing it
  // field-by-field is how stale per-phase state used to leak between scenarios
  // -- assigning a fresh SharedState guarantees nothing is forgotten.
  withMutex([&]() { const_cast<SharedState&>(shared) = SharedState{}; });

  logLine("[HIL] State reset.");
}

inline void parseSimInput() {
  static String buf = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      if (buf.startsWith("HDG:")) {
        withMutex([&]() { shared.raw.compassHeadingDeg = buf.substring(4).toFloat(); });
      }
      else if (buf.startsWith("LAT:")) {
        simGpsLat = buf.substring(4).toDouble();
      }
      else if (buf.startsWith("LON:")) {
        simGpsLon = buf.substring(4).toDouble();
      }
      else if (buf.startsWith("FIX:")) {
        simGpsFix = buf.substring(4).toInt() == 1;
      }
      else if (buf.startsWith("ALT:")) {
        withMutex([&]() { shared.raw.baroAltitudeFt = buf.substring(4).toFloat(); });
      }
      else if (buf.startsWith("MISSION:")) {
        bool fixNow;
        withMutex([&]() { fixNow = shared.raw.gps.fix; });
        if (fixNow) transitionTo(PHASE_MISSION);
      }
      // Fake RC switch commands -- call the exact same handlers the real
      // crsfTask calls on a real CRSF frame. Each CRSFSTART:1 line is one
      // simulated LOW->HIGH edge (send it once per "press", not held).
      else if (buf.startsWith("CRSFSTART:")) {
        if (buf.substring(10).toInt() == 1) crsfHandleStart();
      }
      else if (buf.startsWith("CRSFSTOP:")) {
        if (buf.substring(9).toInt() == 1) crsfHandleStop();
      }
      // HIL runner ping -- confirms firmware is alive and setup() has completed.
      else if (buf.startsWith("PING:")) {
        logLine("[HIL] Ready.");
      }
      else if (buf.startsWith("RESET:")) {
        resetSimState();
      }
      else if (buf.startsWith("ALLOW:")) {
        String allowed = buf.substring(6);
        for (int i = PHASE_PARKED; i <= PHASE_LANDED; ++i) {
          FlightPhase phase = static_cast<FlightPhase>(i);
          if (allowed == phaseName(phase) && hilGatePending && hilGateNext == phase) {
            hilGateApproved = true;
            logLine(String("[HIL_GATE] approved=") + allowed);
            break;
          }
        }
      }
      // On-demand motor telemetry -- queried by the harness so it never
      // collides on the wire with async safety/phase log lines.
      else if (buf.startsWith("MOTOR?")) {
        Dashboard_RTL m;
        withMutex([&]() { m = shared.dashboard_rtl; });
        logLine("[MOTOR] base=" + String(m.baseThrottle, 2) +
                " roll=" + String(m.rollCorrection, 3) +
                " pitch=" + String(m.pitchCorrection, 3));
      }
      // Machine-readable state read-back. Tests use this, not human log lines,
      // because a dropped CDC byte could make a real event look like a missing
      // one.
      else if (buf.startsWith("STATUS?")) {
        String requestId = buf.substring(7);
        if (requestId.length() == 0) requestId = "0";
        FlightPhase phase;
        RTLState rtlState;
        bool gatePending;
        FlightPhase gateNext;
        TransitionReason transitionReason;
        int waypoint;
        withMutex([&]() {
          phase = shared.phase;
          rtlState = shared.trip_rtl.state;
          gatePending = hilGatePending;
          gateNext = hilGateNext;
          transitionReason = shared.transitionReason;
          waypoint = shared.trip_mission.currentWP;
        });
        TransitionReason reason = gatePending ? hilGateReason : transitionReason;
        logLine("[STATUS] id=" + requestId +
                " phase=" + String(phaseName(phase)) +
                " rtl=" + String(rtlState == RTL_CLIMB ? "CLIMB" :
                                    rtlState == RTL_RETURN ? "RETURN" : "SETTLE") +
                " gate=" + String(gatePending ? phaseName(gateNext) : "NONE") +
                " reason=" + reasonName(reason) +
                " wp=" + String(waypoint));
      }
      buf = "";
    } else if (c != '\r') {
      buf += c;
    }
  }
}

#endif // WOKWI_SIM
