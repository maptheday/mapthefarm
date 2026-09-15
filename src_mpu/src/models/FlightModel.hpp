#pragma once

// Domain vocabulary for the flight controller. This header intentionally has
// no ESP32, serial, sensor, or task dependencies.

enum FlightPhase {
  PHASE_PARKED,
  PHASE_RAISE,
  PHASE_HOLD,
  PHASE_MISSION,
  // RTL (return to launch) is three independent phases, entered in order by a
  // failsafe. Each one is a normal, self-contained, individually-triggerable
  // phase -- no "phase inside a phase".
  PHASE_RTL_CLIMB,   // rise to a safe altitude
  PHASE_RTL_RETURN,  // fly back over the launch point
  PHASE_RTL_SETTLE,  // pause a few seconds, then land
  PHASE_HOVER_SETTLE,
  PHASE_LANDING,
  PHASE_LANDED,
  PHASE_CALIBRATE,  // ground maintenance: sensor calibration (motors off)
  PHASE_MANUAL      // pilot flies by RC sticks (sticks drive the Cruise setpoints)
  // NOTE: new phases go at the END -- the registry table in PhaseRegistry.hpp
  // is indexed by this enum's order.
};

enum TransitionReason {
  REASON_NONE,
  REASON_OPERATOR_START,
  REASON_TAKEOFF_COMPLETE,
  REASON_MAX_FLIGHT_TIME,
  REASON_GEOFENCE,
  REASON_GPS_LOSS,
  REASON_MISSION_COMPLETE,
  REASON_RTL_CLIMB_COMPLETE,  // RTL_CLIMB reached altitude -> RTL_RETURN
  REASON_RTL_ARRIVED,         // RTL_RETURN reached launch -> RTL_SETTLE
  REASON_RTL_COMPLETE,        // RTL_SETTLE done -> LANDING
  REASON_HOVER_COMPLETE,
  REASON_TOUCHDOWN,
  REASON_EMERGENCY_STOP,
  REASON_CALIBRATION_COMPLETE,
  REASON_MANUAL_ON,   // pilot took manual stick control
  REASON_MANUAL_OFF   // pilot handed control back to auto-hover
};

inline const char* phaseName(FlightPhase phase) {
  switch (phase) {
    case PHASE_PARKED:       return "PARKED";
    case PHASE_RAISE:        return "RAISE";
    case PHASE_HOLD:         return "HOLD";
    case PHASE_MISSION:      return "MISSION";
    case PHASE_RTL_CLIMB:    return "RTL_CLIMB";
    case PHASE_RTL_RETURN:   return "RTL_RETURN";
    case PHASE_RTL_SETTLE:   return "RTL_SETTLE";
    case PHASE_HOVER_SETTLE: return "HOVER_SETTLE";
    case PHASE_LANDING:      return "LANDING";
    case PHASE_LANDED:       return "LANDED";
    case PHASE_CALIBRATE:    return "CALIBRATE";
    case PHASE_MANUAL:       return "MANUAL";
  }
  return "UNKNOWN";
}

inline bool phaseFlightEnabled(FlightPhase phase) {
  return phase != PHASE_PARKED && phase != PHASE_LANDED && phase != PHASE_CALIBRATE;
}

inline const char* reasonName(TransitionReason reason) {
  switch (reason) {
    case REASON_NONE:             return "NONE";
    case REASON_OPERATOR_START:   return "OPERATOR_START";
    case REASON_TAKEOFF_COMPLETE: return "TAKEOFF_COMPLETE";
    case REASON_MAX_FLIGHT_TIME:  return "MAX_FLIGHT_TIME";
    case REASON_GEOFENCE:         return "GEOFENCE";
    case REASON_GPS_LOSS:         return "GPS_LOSS";
    case REASON_MISSION_COMPLETE: return "MISSION_COMPLETE";
    case REASON_RTL_CLIMB_COMPLETE: return "RTL_CLIMB_COMPLETE";
    case REASON_RTL_ARRIVED:      return "RTL_ARRIVED";
    case REASON_RTL_COMPLETE:     return "RTL_COMPLETE";
    case REASON_HOVER_COMPLETE:   return "HOVER_COMPLETE";
    case REASON_TOUCHDOWN:        return "TOUCHDOWN";
    case REASON_EMERGENCY_STOP:   return "EMERGENCY_STOP";
    case REASON_CALIBRATION_COMPLETE: return "CALIBRATION_COMPLETE";
    case REASON_MANUAL_ON:        return "MANUAL_ON";
    case REASON_MANUAL_OFF:       return "MANUAL_OFF";
  }
  return "UNKNOWN";
}