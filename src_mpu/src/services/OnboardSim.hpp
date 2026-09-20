#pragma once

// ============================================================================
// ONBOARD SIM -- software-in-the-loop (SITL) that runs ENTIRELY ON THE ESP.
//
// This is the on-chip cousin of the laptop RotorPy bridge. Instead of shipping
// motor values out over USB to a physics model on a laptop (serial latency, two
// async clocks, jitter), the physics runs right here in the flight loop using
// the QuadSim library. The firmware's REAL controller flies against REAL physics
// at the REAL 200 Hz rate, one clock, no serial in the loop -- so the recorded
// flight is crisp and precise like the actual aircraft, not the wandering HIL
// artifact.
//
// It is a SERVICE, not a phase: it writes fake sensor readings (attitude, alt,
// GPS) into shared.raw, so the phase logic is untouched and stays fully
// isolated. Built only under SIM (the one and only sim build flag).
//
// Frame mapping (LOCKED -- validated on the laptop in
// lib/QuadSim/examples/nav_check.cpp, and identical to the RotorPy calibration):
//   reported roll  = +quadsim_roll        (stability: firmware +roll = banked right)
//   reported pitch = -quadsim_pitch        (stability: firmware +pitch = nose up)
//   reported north = -quadsim_pos[0]       (QuadSim body-Forward faces world +x, so
//   reported east  = -quadsim_pos[1]        the nav axes are swapped AND flipped)
//   yaw is pinned to 0 in the sim; heading is fed toward the current target so
//   the firmware's yaw controller stays quiet and the flat north->pitch /
//   east->roll nav mapping holds.
//
// It also drives the mission autonomously (fakes the START switch) and logs the
// whole flight to LittleFS as CSV; DUMPLOG streams it back over USB.
// ============================================================================

#include <Arduino.h>

#ifdef SIM

#include <LittleFS.h>
#include <math.h>
#include <QuadSim.h>                    // the on-chip physics library (lib/QuadSim)

#include "../state/PhaseState.hpp"      // shared, withMutex
#include "../state/FlightConfig.hpp"    // waypoints, HOVER_THROTTLE_FF
#include "../models/FlightModel.hpp"    // FlightPhase
#include "../services/NavMath.hpp"      // gpsBearing, getMissionWaypoint
#include "../services/Motors.hpp"       // motors.lastMix
#include "../services/RcInput.hpp"      // crsfHandleStart
#include "../services/Log.hpp"          // logLine

// Launch point: the farmer's field corner 1. Must match where the firmware
// captures its launch GPS at arm time (WAYPOINTS' implied launch in FlightConfig).
static const double ONBOARD_HOME_LAT = 35.948305;
static const double ONBOARD_HOME_LON = -78.241377;

static const float  FT_PER_M       = 3.28084f;
static const double M_PER_DEG_LAT  = 111320.0;
#define ONBOARD_LOG_PATH "/flight.csv"

// ---- the physics model + bookkeeping (single instance, this build only) -----
namespace onboard {

// A ~4:1 thrust/weight airframe so hover sits near HOVER_THROTTLE_FF (~0.5),
// matching the firmware's feed-forward assumption.
inline quadsim::Params airframe() {
  quadsim::Params p = quadsim::Params::generic();
  p.maxRotorSpeed = sqrtf(4.0f * p.mass * quadsim::kGravity / (4.0f * p.kThrust));
  return p;
}

inline quadsim::Multirotor& uav() {
  static quadsim::Multirotor u(airframe(), quadsim::State::level(0.0f));
  return u;
}

inline File& logFile() { static File f; return f; }
inline unsigned long& t0Ms()      { static unsigned long v = 0; return v; }
inline unsigned long& lastStartMs(){ static unsigned long v = 0; return v; }
inline unsigned long& lastLogMs()  { static unsigned long v = 0; return v; }

// --- test-scenario state (set at boot by the SCENARIO: serial command) --------
inline String& scenario()          { static String s = "full"; return s; }  // which test
inline bool& gpsLossFault()        { static bool b = false; return b; }      // drop GPS fix
inline unsigned long& kickUntilMs(){ static unsigned long v = 0; return v; } // hold a tilt disturbance until this ms
inline bool& scenarioActed()       { static bool b = false; return b; }      // HOLD trigger fired
inline bool& scenarioDone()        { static bool b = false; return b; }      // terminal reached
inline unsigned long& scenarioMs() { static unsigned long v = 0; return v; } // when the trigger fired

}  // namespace onboard

// Select a test scenario (see scenarios_esp/). Called once at boot from the
// SCENARIO: serial command, before takeoff. "full" (the default) just flies the
// whole patrol; the others inject one fault to exercise a failsafe / mode.
inline void onboardSimSetScenario(const String& s) {
  onboard::scenario() = s;
  if (s == "geofence") GEOFENCE_RADIUS_M  = 40.0f;     // shrink the fence -> trips soon after takeoff
  if (s == "timeout")  MAX_FLIGHT_TIME_MS = 18000UL;   // ~18 s of flight -> max-time RTL
  logLine(String("[SCENARIO] selected: ") + s);
}

// ---------------------------------------------------------------------------
// setup(): bring up the filesystem, truncate a fresh log, seed the GPS at home.
// ---------------------------------------------------------------------------
inline void onboardSimBegin() {
  if (!LittleFS.begin(true)) {
    logLine("[ONBOARD] LittleFS mount FAILED");
  } else {
    onboard::logFile() = LittleFS.open(ONBOARD_LOG_PATH, "w");
    if (onboard::logFile()) {
      onboard::logFile().printf("# home_lat=%.6f home_lon=%.6f\n", ONBOARD_HOME_LAT, ONBOARD_HOME_LON);
      onboard::logFile().println("t_s,phase,north_m,east_m,up_ft,roll_deg,pitch_deg,dist_m,wp");
      onboard::logFile().flush();
    }
    logLine("[ONBOARD] log open " ONBOARD_LOG_PATH);
  }
  double lonScale = M_PER_DEG_LAT * cos(ONBOARD_HOME_LAT * DEG_TO_RAD);
  (void)lonScale;
  // Seed a GPS fix at home so RAISE captures the right launch point.
  withMutex([&]() {
    shared.raw.gps.lat       = ONBOARD_HOME_LAT;
    shared.raw.gps.lon       = ONBOARD_HOME_LON;
    shared.raw.gps.fix       = true;
    shared.raw.gps.sats      = 10;
    shared.raw.gps.lastFixMs = millis();
    shared.raw.baroAltitudeFt = 0.0f;
    shared.raw.compassHeadingDeg = 0.0f;
  });
  onboard::t0Ms() = millis();
  onboard::lastStartMs() = 0;
  logLine("[ONBOARD] on-chip SITL ready -- autonomous flight.");
}

// ---------------------------------------------------------------------------
// physicsTick (200 Hz): step the physics with the LAST commanded mix, then write
// the resulting attitude / altitude / GPS back into shared.raw as fake sensors.
// Runs BEFORE the phase's physicsTick, so the controller reacts to fresh state.
// ---------------------------------------------------------------------------
inline void onboardSimStep(float dt) {
  if (dt <= 0.0f || dt > 0.5f) dt = 0.005f;

  // Last four throttles the flight code commanded (0 while disarmed on ground).
  MotorMix mix;
  withMutex([&]() { mix = motors.lastMix; });
  float m[4] = { mix.m1, mix.m2, mix.m3, mix.m4 };

  quadsim::Multirotor& u = onboard::uav();
  u.step(m, dt);

  // Pin yaw to 0: rebuild the quaternion from roll+pitch only, zero yaw rate,
  // so the drone flies world-aligned and the flat nav mapping holds.
  float qr, qp, qy; u.rpyDeg(qr, qp, qy);
  // Stabilization scenario: hold an attitude disturbance (a gust that pins the
  // drone at ~22 deg roll) for a short window, then release and let the controller
  // recover. Sustained so the 10 Hz log clearly captures the excursion. The body
  // rates are zeroed while held so no momentum builds up -- on release it recovers
  // cleanly from 22 deg instead of getting flung past level.
  bool kicking = millis() < onboard::kickUntilMs();
  if (kicking) qr = 22.0f;
  quadsim::State& s = u.mutableState();
  float hr = qr * DEG_TO_RAD * 0.5f, hp = qp * DEG_TO_RAD * 0.5f;
  float cr = cosf(hr), sr = sinf(hr), cp = cosf(hp), sp = sinf(hp);
  s.quat[0] = cr * cp; s.quat[1] = sr * cp; s.quat[2] = cr * sp; s.quat[3] = -sr * sp;
  s.omega[2] = 0.0f;
  if (kicking) { s.omega[0] = 0.0f; s.omega[1] = 0.0f; }

  // Reported sensors (LOCKED frame mapping -- see header).
  float roll  = +qr;
  float pitch = -qp;
  float north = -s.pos[0];
  float east  = -s.pos[1];
  float upFt  = u.altitude() * FT_PER_M;

  double lat = ONBOARD_HOME_LAT + north / M_PER_DEG_LAT;
  double lon = ONBOARD_HOME_LON + east / (M_PER_DEG_LAT * cos(ONBOARD_HOME_LAT * DEG_TO_RAD));

  // GPS-loss scenario: report no fix and stop refreshing the fix time, so the
  // GPS-loss failsafe trips (aborts to LANDING) after GPS_LOSS_ABORT_MS.
  bool gl = onboard::gpsLossFault();
  withMutex([&]() {
    shared.raw.imu.gyroX = roll;   // fused roll  (Imu.hpp naming quirk)
    shared.raw.imu.gyroY = pitch;  // fused pitch
    shared.raw.imu.gyroZ = 0.0f;   // yaw rate (pinned)
    shared.raw.baroAltitudeFt = upFt;
    shared.raw.gps.lat       = lat;
    shared.raw.gps.lon       = lon;
    shared.raw.gps.fix       = gl ? false : true;
    shared.raw.gps.sats      = gl ? 0 : 10;
    if (!gl) shared.raw.gps.lastFixMs = millis();
  });
}

// ---------------------------------------------------------------------------
// navTick (10 Hz): autonomously fly the mission (fake the START switch), keep
// the compass heading pointed at the current target so yaw stays quiet, and
// append one CSV sample to the flight log.
// ---------------------------------------------------------------------------
inline void onboardSimNav() {
  unsigned long now = millis();
  unsigned long t   = now - onboard::t0Ms();

  FlightPhase  phase;
  RawGpsReading gps;
  float roll, pitch, upFt;
  int   wp;
  double launchLat, launchLon;
  withMutex([&]() {
    phase = shared.phase;
    gps   = shared.raw.gps;
    roll  = shared.raw.imu.gyroX;
    pitch = shared.raw.imu.gyroY;
    upFt  = shared.raw.baroAltitudeFt;
    wp        = shared.trip_mission.currentWP;
    launchLat = shared.trip_mission.launchLat;
    launchLon = shared.trip_mission.launchLon;
  });

  const String sc = onboard::scenario();
  // "full", "geofence" and "timeout" fly the real mission; the others act in HOLD.
  const bool missionScenario = (sc == "full" || sc == "geofence" || sc == "timeout");

  // --- autostart: PARKED -> RAISE, then (mission scenarios) HOLD -> MISSION.
  // t>2500 leaves time for the SCENARIO: command to arrive before takeoff.
  if (t > 2500 && now - onboard::lastStartMs() > 1500) {
    if (phase == PHASE_PARKED) {
      crsfHandleStart();
      onboard::lastStartMs() = now;
    } else if (phase == PHASE_HOLD && missionScenario) {
      crsfHandleStart();
      onboard::lastStartMs() = now;
    }
  }

  // --- fault scenarios that act once, on reaching HOLD -----------------------
  if (phase == PHASE_HOLD && !onboard::scenarioActed() && t > 2500) {
    if (sc == "gpsloss") {
      onboard::gpsLossFault() = true;   // GPS drops -> abort to LANDING
      onboard::scenarioActed() = true;
      logLine("[SCENARIO] GPS fix dropped");
    } else if (sc == "stab") {
      onboard::kickUntilMs() = now + 500;  // hold a ~22 deg roll for 0.5 s, then recover
      onboard::scenarioActed() = true;
      onboard::scenarioMs() = now;
      logLine("[SCENARIO] attitude kick applied");
    } else if (sc == "manual") {
      crsfHandleManualOn();             // pilot takes the sticks
      onboard::scenarioActed() = true;
      onboard::scenarioMs() = now;
      logLine("[SCENARIO] manual control on");
    }
  }

  // --- manual scenario: fly forward on the sticks, then stop -----------------
  if (sc == "manual" && onboard::scenarioActed() && !onboard::scenarioDone()) {
    unsigned long e = now - onboard::scenarioMs();
    if (phase == PHASE_MANUAL && e < 5000) {
      withMutex([&]() { shared.sticks.throttle = 0.5f; shared.sticks.pitch = 0.5f;
                        shared.sticks.roll = 0.0f; shared.sticks.yaw = 0.0f; });
    } else if (e >= 5000) {
      withMutex([&]() { shared.sticks.throttle = 0.5f; shared.sticks.pitch = 0.0f;
                        shared.sticks.roll = 0.0f; shared.sticks.yaw = 0.0f; });
      crsfHandleStop();
      onboard::scenarioDone() = true;
      logLine("[SCENARIO] ===SCENARIO_DONE===");
    }
  }
  // --- stabilization scenario: after the kick, give it time to recover, stop --
  if (sc == "stab" && onboard::scenarioActed() && !onboard::scenarioDone()
      && now - onboard::scenarioMs() > 6000) {
    crsfHandleStop();
    onboard::scenarioDone() = true;
    logLine("[SCENARIO] ===SCENARIO_DONE===");
  }

  // --- keep heading pointed at the current target so the yaw loop is quiet ---
  double tgtLat = 0, tgtLon = 0; bool haveTgt = false;
  if (phase == PHASE_MISSION) {
    Waypoint w = getMissionWaypoint(wp, launchLat, launchLon);
    tgtLat = w.lat; tgtLon = w.lon; haveTgt = true;
  } else if (phase == PHASE_RTL_RETURN) {
    tgtLat = launchLat; tgtLon = launchLon; haveTgt = true;
  }
  if (haveTgt) {
    float brg = gpsBearing(gps.lat, gps.lon, tgtLat, tgtLon);
    withMutex([&]() { shared.raw.compassHeadingDeg = brg; });
  }

  // --- log one sample at ~10 Hz ---
  if (onboard::logFile() && now - onboard::lastLogMs() >= 100) {
    onboard::lastLogMs() = now;
    float north = (gps.lat - ONBOARD_HOME_LAT) * M_PER_DEG_LAT;
    float east  = (gps.lon - ONBOARD_HOME_LON) * (M_PER_DEG_LAT * cos(ONBOARD_HOME_LAT * DEG_TO_RAD));
    float dist  = sqrtf(north * north + east * east);
    onboard::logFile().printf("%.2f,%s,%.1f,%.1f,%.1f,%.2f,%.2f,%.1f,%d\n",
        t / 1000.0f, phaseName(phase), north, east, upFt, roll, pitch, dist, wp);
    if (phase == PHASE_LANDED) onboard::logFile().flush();
  }
}

// ---------------------------------------------------------------------------
// DUMPLOG: stream the recorded CSV back over USB between clear markers so the
// laptop recorder can capture the on-chip flight and build the viz from it.
// ---------------------------------------------------------------------------
inline void onboardSimDumpLog() {
  if (onboard::logFile()) onboard::logFile().flush();
  File f = LittleFS.open(ONBOARD_LOG_PATH, "r");
  if (!f) { logLine("[ONBOARD] no log to dump"); return; }
  Serial.println("===ONBOARD_LOG_START===");
  while (f.available()) Serial.write(f.read());
  Serial.println("===ONBOARD_LOG_END===");
  f.close();
}

#endif  // SIM
