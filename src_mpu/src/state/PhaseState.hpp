#pragma once

// ============================================================================
// THE REPOSITORY
// ----------------------------------------------------------------------------
// ELI5: this file is the drone's single "notebook". Every part of the code
// (sensors, phases, the web page) reads and writes the SAME notebook so nobody
// disagrees about what's happening. Because two CPU cores scribble in it at the
// same time, you must always go through withMutex() -- that's the pen that only
// one core can hold at a time, so two writes never smear together.
//
// The notebook is split into one little section PER FLIGHT PHASE. Each phase
// owns three blocks:
//   Dashboard_X -> numbers we SHOW (telemetry / web page)          [outputs]
//   Cruise_X    -> what the phase is AIMING for (target alt, etc.) [setpoints]
//   Trip_X      -> bookkeeping for this leg (arm time, waypoint #)  [memory]
//
// Rule of thumb when you add a field:
//   "Does the phase need this to make its next decision?" -> Trip_
//   "Am I aiming at this?"                                 -> Cruise_
//   "Do I just want to see it?"                            -> Dashboard_
// (Trip_ is a phase's private working memory. It can still publish a friendly
//  derived view via writeTelemetry() -- e.g. currentWP -> navWaypoint.)
//
// Keeping each phase's state separate is on purpose: a bug in MISSION can only
// touch mission's block, so you can reason about one phase at a time.
// ============================================================================

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "../models/FlightModel.hpp"
#include "../models/SensorTypes.hpp"

// --- PARKED ---
struct Dashboard_Parked {
  float altitudeFt = 0.0f;
  float roll       = 0.0f;
  float pitch      = 0.0f;
  float yaw        = 0.0f;
};
struct Cruise_Parked {};
struct Trip_Parked {};

// --- RAISE (Takeoff) ---
struct Dashboard_Raise {
  float altitudeFt      = 0.0f;
  float roll            = 0.0f;
  float pitch           = 0.0f;
  float yaw             = 0.0f;
  float compassHeading  = 0.0f;
  float m1              = 0.0f;
  float m2              = 0.0f;
  float m3              = 0.0f;
  float m4              = 0.0f;
  float baseThrottle    = 0.0f;
  float rollCorrection  = 0.0f;
  float pitchCorrection = 0.0f;
  Dashboard_Raise& operator=(const volatile Dashboard_Raise& o) {
    altitudeFt=o.altitudeFt; roll=o.roll; pitch=o.pitch; yaw=o.yaw;
    compassHeading=o.compassHeading;
    m1=o.m1; m2=o.m2; m3=o.m3; m4=o.m4;
    baseThrottle=o.baseThrottle; rollCorrection=o.rollCorrection;
    pitchCorrection=o.pitchCorrection;
    return *this;
  }
};
struct Cruise_Raise {
  float targetAltFt      = 0.0f;
  float targetRollDeg    = 0.0f;
  float targetPitchDeg   = 0.0f;
  float yawTargetHeading = 0.0f;
  Cruise_Raise& operator=(const volatile Cruise_Raise& o) {
    targetAltFt=o.targetAltFt; targetRollDeg=o.targetRollDeg;
    targetPitchDeg=o.targetPitchDeg; yawTargetHeading=o.yawTargetHeading;
    return *this;
  }
};
struct Trip_Raise {
  unsigned long armedAtMs = 0;
  double        launchLat = 0.0;
  double        launchLon = 0.0;
  Trip_Raise& operator=(const volatile Trip_Raise& o) {
    armedAtMs=o.armedAtMs; launchLat=o.launchLat; launchLon=o.launchLon;
    return *this;
  }
};

// --- HOLD ---
struct Dashboard_Hold {
  float altitudeFt      = 0.0f;
  float roll            = 0.0f;
  float pitch           = 0.0f;
  float yaw             = 0.0f;
  float compassHeading  = 0.0f;
  float m1              = 0.0f;
  float m2              = 0.0f;
  float m3              = 0.0f;
  float m4              = 0.0f;
  float baseThrottle    = 0.0f;
  float rollCorrection  = 0.0f;
  float pitchCorrection = 0.0f;
  Dashboard_Hold& operator=(const volatile Dashboard_Hold& o) {
    altitudeFt=o.altitudeFt; roll=o.roll; pitch=o.pitch; yaw=o.yaw;
    compassHeading=o.compassHeading;
    m1=o.m1; m2=o.m2; m3=o.m3; m4=o.m4;
    baseThrottle=o.baseThrottle; rollCorrection=o.rollCorrection;
    pitchCorrection=o.pitchCorrection;
    return *this;
  }
};
struct Cruise_Hold {
  float targetAltFt      = 10.0f;
  float targetRollDeg    = 0.0f;
  float targetPitchDeg   = 0.0f;
  float yawTargetHeading = 0.0f;
  Cruise_Hold& operator=(const volatile Cruise_Hold& o) {
    targetAltFt=o.targetAltFt; targetRollDeg=o.targetRollDeg;
    targetPitchDeg=o.targetPitchDeg; yawTargetHeading=o.yawTargetHeading;
    return *this;
  }
};
struct Trip_Hold {
  unsigned long armedAtMs = 0;
  double        launchLat = 0.0;
  double        launchLon = 0.0;
  Trip_Hold& operator=(const volatile Trip_Hold& o) {
    armedAtMs=o.armedAtMs; launchLat=o.launchLat; launchLon=o.launchLon;
    return *this;
  }
};

// --- MISSION ---
struct Dashboard_Mission {
  float  altitudeFt      = 0.0f;
  float  roll            = 0.0f;
  float  pitch           = 0.0f;
  float  yaw             = 0.0f;
  float  compassHeading  = 0.0f;
  double gpsLat          = 0.0;
  double gpsLon          = 0.0;
  bool   gpsFix          = false;
  int    gpsSats         = 0;
  float  distToWP        = 0.0f;
  float  bearingToWP     = 0.0f;
  float  m1              = 0.0f;
  float  m2              = 0.0f;
  float  m3              = 0.0f;
  float  m4              = 0.0f;
  float  baseThrottle    = 0.0f;
  float  rollCorrection  = 0.0f;
  float  pitchCorrection = 0.0f;
  Dashboard_Mission& operator=(const volatile Dashboard_Mission& o) {
    altitudeFt=o.altitudeFt; roll=o.roll; pitch=o.pitch; yaw=o.yaw;
    compassHeading=o.compassHeading;
    gpsLat=o.gpsLat; gpsLon=o.gpsLon; gpsFix=o.gpsFix; gpsSats=o.gpsSats;
    distToWP=o.distToWP; bearingToWP=o.bearingToWP;
    m1=o.m1; m2=o.m2; m3=o.m3; m4=o.m4;
    baseThrottle=o.baseThrottle; rollCorrection=o.rollCorrection;
    pitchCorrection=o.pitchCorrection;
    return *this;
  }
};
struct Cruise_Mission {
  float targetAltFt      = 10.0f;
  float targetRollDeg    = 0.0f;
  float targetPitchDeg   = 0.0f;
  float yawTargetHeading = 0.0f;
  Cruise_Mission& operator=(const volatile Cruise_Mission& o) {
    targetAltFt=o.targetAltFt; targetRollDeg=o.targetRollDeg;
    targetPitchDeg=o.targetPitchDeg; yawTargetHeading=o.yawTargetHeading;
    return *this;
  }
};
struct Trip_Mission {
  unsigned long armedAtMs     = 0;
  int           currentWP     = 0;
  int           waypointCount = 0;
  bool          active        = false;
  double        launchLat     = 0.0;
  double        launchLon     = 0.0;
  Trip_Mission& operator=(const volatile Trip_Mission& o) {
    armedAtMs=o.armedAtMs; currentWP=o.currentWP;
    waypointCount=o.waypointCount; active=o.active;
    launchLat=o.launchLat; launchLon=o.launchLon;
    return *this;
  }
};

// --- RTL ---
enum RTLState {
  RTL_CLIMB,
  RTL_RETURN,
  RTL_SETTLE
};

struct Dashboard_RTL {
  float altitudeFt      = 0.0f;
  float roll            = 0.0f;
  float pitch           = 0.0f;
  float yaw             = 0.0f;
  float compassHeading  = 0.0f;
  float m1              = 0.0f;
  float m2              = 0.0f;
  float m3              = 0.0f;
  float m4              = 0.0f;
  float baseThrottle    = 0.0f;
  float rollCorrection  = 0.0f;
  float pitchCorrection = 0.0f;
  Dashboard_RTL& operator=(const volatile Dashboard_RTL& o) {
    altitudeFt=o.altitudeFt; roll=o.roll; pitch=o.pitch; yaw=o.yaw;
    compassHeading=o.compassHeading;
    m1=o.m1; m2=o.m2; m3=o.m3; m4=o.m4;
    baseThrottle=o.baseThrottle; rollCorrection=o.rollCorrection;
    pitchCorrection=o.pitchCorrection;
    return *this;
  }
};
struct Cruise_RTL {
  float targetAltFt      = 10.0f;
  float targetRollDeg    = 0.0f;
  float targetPitchDeg   = 0.0f;
  float yawTargetHeading = 0.0f;
  Cruise_RTL& operator=(const volatile Cruise_RTL& o) {
    targetAltFt=o.targetAltFt; targetRollDeg=o.targetRollDeg;
    targetPitchDeg=o.targetPitchDeg; yawTargetHeading=o.yawTargetHeading;
    return *this;
  }
};
struct Trip_RTL {
  unsigned long armedAtMs     = 0;
  RTLState      state         = RTL_CLIMB;
  double        launchLat     = 0.0;
  double        launchLon     = 0.0;
  unsigned long settleStartMs = 0;
  Trip_RTL& operator=(const volatile Trip_RTL& o) {
    armedAtMs=o.armedAtMs; state=o.state;
    launchLat=o.launchLat; launchLon=o.launchLon;
    settleStartMs=o.settleStartMs;
    return *this;
  }
};

// --- HOVER SETTLE ---
struct Dashboard_HoverSettle {
  float altitudeFt      = 0.0f;
  float roll            = 0.0f;
  float pitch           = 0.0f;
  float yaw             = 0.0f;
  float compassHeading  = 0.0f;
  float m1              = 0.0f;
  float m2              = 0.0f;
  float m3              = 0.0f;
  float m4              = 0.0f;
  float baseThrottle    = 0.0f;
  float rollCorrection  = 0.0f;
  float pitchCorrection = 0.0f;
  Dashboard_HoverSettle& operator=(const volatile Dashboard_HoverSettle& o) {
    altitudeFt=o.altitudeFt; roll=o.roll; pitch=o.pitch; yaw=o.yaw;
    compassHeading=o.compassHeading;
    m1=o.m1; m2=o.m2; m3=o.m3; m4=o.m4;
    baseThrottle=o.baseThrottle; rollCorrection=o.rollCorrection;
    pitchCorrection=o.pitchCorrection;
    return *this;
  }
};
struct Cruise_HoverSettle {
  float targetAltFt      = 10.0f;
  float targetRollDeg    = 0.0f;
  float targetPitchDeg   = 0.0f;
  float yawTargetHeading = 0.0f;
  Cruise_HoverSettle& operator=(const volatile Cruise_HoverSettle& o) {
    targetAltFt=o.targetAltFt; targetRollDeg=o.targetRollDeg;
    targetPitchDeg=o.targetPitchDeg; yawTargetHeading=o.yawTargetHeading;
    return *this;
  }
};
struct Trip_HoverSettle {
  unsigned long enteredAtMs = 0;
};

// --- LANDING ---
struct Dashboard_Landing {
  float altitudeFt      = 0.0f;
  float roll            = 0.0f;
  float pitch           = 0.0f;
  float yaw             = 0.0f;
  float compassHeading  = 0.0f;
  float m1              = 0.0f;
  float m2              = 0.0f;
  float m3              = 0.0f;
  float m4              = 0.0f;
  float baseThrottle    = 0.0f;
  float rollCorrection  = 0.0f;
  float pitchCorrection = 0.0f;
  Dashboard_Landing& operator=(const volatile Dashboard_Landing& o) {
    altitudeFt=o.altitudeFt; roll=o.roll; pitch=o.pitch; yaw=o.yaw;
    compassHeading=o.compassHeading;
    m1=o.m1; m2=o.m2; m3=o.m3; m4=o.m4;
    baseThrottle=o.baseThrottle; rollCorrection=o.rollCorrection;
    pitchCorrection=o.pitchCorrection;
    return *this;
  }
};
struct Cruise_Landing {
  float targetAltFt      = 0.0f;
  float targetRollDeg    = 0.0f;
  float targetPitchDeg   = 0.0f;
  float yawTargetHeading = 0.0f;
  Cruise_Landing& operator=(const volatile Cruise_Landing& o) {
    targetAltFt=o.targetAltFt; targetRollDeg=o.targetRollDeg;
    targetPitchDeg=o.targetPitchDeg; yawTargetHeading=o.yawTargetHeading;
    return *this;
  }
};
struct Trip_Landing {};

// --- LANDED ---
struct Dashboard_Landed {
  float altitudeFt = 0.0f;
};
struct Cruise_Landed {};
struct Trip_Landed {};

// --- CALIBRATE (compass) ---
struct Dashboard_Calibrate {
  float progressPct = 0.0f;
};
struct Cruise_Calibrate {}; // nothing to aim at -- motors are off
struct Trip_Calibrate {
  unsigned long enteredAtMs = 0;
};

// ============================================================================
// SHARED STATE -- the whole notebook, one struct.
// ============================================================================
struct SharedState {
  FlightPhase phase = PHASE_PARKED;
  TransitionReason transitionReason = REASON_NONE;
  RawSensors  raw;

  Dashboard_Parked      dashboard_parked;
  Cruise_Parked         cruise_parked;
  Trip_Parked           trip_parked;

  Dashboard_Raise       dashboard_raise;
  Cruise_Raise          cruise_raise;
  Trip_Raise            trip_raise;

  Dashboard_Hold        dashboard_hold;
  Cruise_Hold           cruise_hold;
  Trip_Hold             trip_hold;

  Dashboard_Mission     dashboard_mission;
  Cruise_Mission        cruise_mission;
  Trip_Mission          trip_mission;

  Dashboard_RTL         dashboard_rtl;
  Cruise_RTL            cruise_rtl;
  Trip_RTL              trip_rtl;

  Dashboard_HoverSettle dashboard_hoverSettle;
  Cruise_HoverSettle    cruise_hoverSettle;
  Trip_HoverSettle      trip_hoverSettle;

  Dashboard_Landing     dashboard_landing;
  Cruise_Landing        cruise_landing;
  Trip_Landing          trip_landing;

  Dashboard_Landed      dashboard_landed;
  Cruise_Landed         cruise_landed;
  Trip_Landed           trip_landed;

  Dashboard_Calibrate   dashboard_calibrate;
  Cruise_Calibrate      cruise_calibrate;
  Trip_Calibrate        trip_calibrate;
};

// ----------------------------------------------------------------------------
// Repository access. The single `shared` instance and the pen (`sharedDataMutex`)
// are DEFINED once in the .ino; everything else just borrows them via `extern`.
// ----------------------------------------------------------------------------
extern SemaphoreHandle_t sharedDataMutex;
extern volatile SharedState shared;

// Grab the pen, run your code, put the pen back. Always touch `shared` inside
// one of these. Do NOT nest withMutex() calls -- the pen is not re-entrant.
template<typename Fn>
void withMutex(Fn fn) {
  if (xSemaphoreTake(sharedDataMutex, portMAX_DELAY) == pdTRUE) {
    fn();
    xSemaphoreGive(sharedDataMutex);
  }
}
