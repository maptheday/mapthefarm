#pragma once

// ============================================================================
// SIM ADAPTER -- the hardware-in-the-loop (HIL) test interface. Sim builds only.
// The Python test runner drives the drone entirely over the USB serial line
// with tiny text commands; this file parses them and speaks the reply protocol:
//   inputs:  HDG:/LAT:/LON:/FIX:/ALT:  inject fake sensor readings
//            CRSFSTART:1 / CRSFSTOP:1   fake the pilot's radio switches
//            CRSFMANUAL:1 / :0          fake the MANUAL switch on/off
//            STICKS:th,ro,pi,ya         fake the four RC sticks (MANUAL mode)
//            MISSION:                   shortcut to start a mission
//            ALLOW:<PHASE>              approve a gated transition
//            RESET:                     wipe state between scenarios
//            PING:                      liveness check
//   queries: STATUS? / MOTOR? / MANUAL? machine-readable state read-back
// Kept out of the real firmware path -- none of this compiles on hardware.
// ============================================================================

#include <Arduino.h>

#ifdef WOKWI_SIM

#include "../state/PhaseState.hpp"     // shared, withMutex, per-phase dashboards
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
      // MANUAL switch: :1 grabs the sticks, :0 hands back to auto-hover. Same
      // handlers the real crsfTask calls on the switch edge.
      else if (buf.startsWith("CRSFMANUAL:")) {
        if (buf.substring(11).toInt() == 1) crsfHandleManualOn();
        else                                crsfHandleManualOff();
      }
      // Fake the four RC sticks for MANUAL mode: "STICKS:throttle,roll,pitch,yaw"
      // (throttle 0..1 with 0.5 centered; roll/pitch/yaw -1..1 centered at 0).
      else if (buf.startsWith("STICKS:")) {
        String p = buf.substring(7);
        int c1 = p.indexOf(',');
        int c2 = p.indexOf(',', c1 + 1);
        int c3 = p.indexOf(',', c2 + 1);
        if (c1 > 0 && c2 > c1 && c3 > c2) {
          float th = p.substring(0, c1).toFloat();
          float ro = p.substring(c1 + 1, c2).toFloat();
          float pi = p.substring(c2 + 1, c3).toFloat();
          float ya = p.substring(c3 + 1).toFloat();
          withMutex([&]() {
            shared.sticks.throttle = th;
            shared.sticks.roll     = ro;
            shared.sticks.pitch    = pi;
            shared.sticks.yaw      = ya;
          });
        }
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
        for (int i = PHASE_PARKED; i <= PHASE_MANUAL; ++i) {  // covers every FlightPhase
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
        // Report the motor mix of whichever phase is currently flying. Each
        // phase writes its own dashboard, so pick the one matching shared.phase.
        float base = 0.0f;
        float roll = 0.0f;
        float pitch = 0.0f;
        withMutex([&]() {
          switch (shared.phase) {
            case PHASE_RAISE:
              base=shared.dashboard_raise.baseThrottle; roll=shared.dashboard_raise.rollCorrection; pitch=shared.dashboard_raise.pitchCorrection; break;
            case PHASE_HOLD:
              base=shared.dashboard_hold.baseThrottle; roll=shared.dashboard_hold.rollCorrection; pitch=shared.dashboard_hold.pitchCorrection; break;
            case PHASE_MISSION:
              base=shared.dashboard_mission.baseThrottle; roll=shared.dashboard_mission.rollCorrection; pitch=shared.dashboard_mission.pitchCorrection; break;
            case PHASE_RTL_CLIMB:
              base=shared.dashboard_rtlClimb.baseThrottle; roll=shared.dashboard_rtlClimb.rollCorrection; pitch=shared.dashboard_rtlClimb.pitchCorrection; break;
            case PHASE_RTL_RETURN:
              base=shared.dashboard_rtlReturn.baseThrottle; roll=shared.dashboard_rtlReturn.rollCorrection; pitch=shared.dashboard_rtlReturn.pitchCorrection; break;
            case PHASE_RTL_SETTLE:
              base=shared.dashboard_rtlSettle.baseThrottle; roll=shared.dashboard_rtlSettle.rollCorrection; pitch=shared.dashboard_rtlSettle.pitchCorrection; break;
            case PHASE_HOVER_SETTLE:
              base=shared.dashboard_hoverSettle.baseThrottle; roll=shared.dashboard_hoverSettle.rollCorrection; pitch=shared.dashboard_hoverSettle.pitchCorrection; break;
            case PHASE_LANDING:
              base=shared.dashboard_landing.baseThrottle; roll=shared.dashboard_landing.rollCorrection; pitch=shared.dashboard_landing.pitchCorrection; break;
            case PHASE_MANUAL:
              base=shared.dashboard_manual.baseThrottle; roll=shared.dashboard_manual.rollCorrection; pitch=shared.dashboard_manual.pitchCorrection; break;
            default: break;  // ground phases (PARKED / LANDED / CALIBRATE): motors off
          }
        });
        logLine("[MOTOR] base=" + String(base, 2) +
                " roll=" + String(roll, 3) +
                " pitch=" + String(pitch, 3));
      }
      // MANUAL-mode read-back: the pilot setpoints the sticks are driving, plus
      // whether position hold has dropped an anchor and the base throttle.
      else if (buf.startsWith("MANUAL?")) {
        Cruise_Manual    c;
        Trip_Manual      t;
        Dashboard_Manual d;
        withMutex([&]() {
          c = shared.cruise_manual;
          t = shared.trip_manual;
          d = shared.dashboard_manual;
        });
        logLine("[MANUAL] targetAlt=" + String(c.targetAltFt, 2) +
                " targetRoll=" + String(c.targetRollDeg, 2) +
                " targetPitch=" + String(c.targetPitchDeg, 2) +
                " yaw=" + String(c.yawTargetHeading, 1) +
                " anchored=" + String(t.anchored ? 1 : 0) +
                " base=" + String(d.baseThrottle, 2));
      }
      // Machine-readable state read-back. Tests use this, not human log lines,
      // because a dropped CDC byte could make a real event look like a missing
      // one.
      else if (buf.startsWith("STATUS?")) {
        String requestId = buf.substring(7);
        if (requestId.length() == 0) requestId = "0";
        FlightPhase phase;
        bool gatePending;
        FlightPhase gateNext;
        TransitionReason transitionReason;
        int waypoint;
        withMutex([&]() {
          phase = shared.phase;
          gatePending = hilGatePending;
          gateNext = hilGateNext;
          transitionReason = shared.transitionReason;
          waypoint = shared.trip_mission.currentWP;
        });
        TransitionReason reason = gatePending ? hilGateReason : transitionReason;
        // The RTL sub-state used to ride along as a separate rtl= field; it is
        // now just the phase name (RTL_CLIMB / RTL_RETURN / RTL_SETTLE).
        logLine("[STATUS] id=" + requestId +
                " phase=" + String(phaseName(phase)) +
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
