#pragma once

// Domain vocabulary for the flight controller. This header intentionally has
// no ESP32, serial, sensor, or task dependencies.

enum FlightPhase {
  PHASE_PARKED,
  PHASE_RAISE,
  PHASE_HOLD,
  PHASE_MISSION,
  PHASE_RTL,
  PHASE_HOVER_SETTLE,
  PHASE_LANDING,
  PHASE_LANDED
};

enum TransitionReason {
  REASON_NONE,
  REASON_OPERATOR_START,
  REASON_TAKEOFF_COMPLETE,
  REASON_MAX_FLIGHT_TIME,
  REASON_GEOFENCE,
  REASON_GPS_LOSS,
  REASON_MISSION_COMPLETE,
  REASON_RTL_COMPLETE,
  REASON_HOVER_COMPLETE,
  REASON_TOUCHDOWN,
  REASON_EMERGENCY_STOP
};

inline const char* phaseName(FlightPhase phase) {
  switch (phase) {
    case PHASE_PARKED:       return "PARKED";
    case PHASE_RAISE:        return "RAISE";
    case PHASE_HOLD:         return "HOLD";
    case PHASE_MISSION:      return "MISSION";
    case PHASE_RTL:          return "RTL";
    case PHASE_HOVER_SETTLE: return "HOVER_SETTLE";
    case PHASE_LANDING:      return "LANDING";
    case PHASE_LANDED:       return "LANDED";
  }
  return "UNKNOWN";
}

inline bool phaseFlightEnabled(FlightPhase phase) {
  return phase != PHASE_PARKED && phase != PHASE_LANDED;
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
    case REASON_RTL_COMPLETE:     return "RTL_COMPLETE";
    case REASON_HOVER_COMPLETE:   return "HOVER_COMPLETE";
    case REASON_TOUCHDOWN:        return "TOUCHDOWN";
    case REASON_EMERGENCY_STOP:   return "EMERGENCY_STOP";
  }
  return "UNKNOWN";
}