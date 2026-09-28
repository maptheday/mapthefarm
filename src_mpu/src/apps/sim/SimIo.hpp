#pragma once

// ============================================================================
// SIM IO -- the on-chip simulator's FlightIo. Software-in-the-loop that runs
// ENTIRELY ON THE ESP.
//
// The flight controller asks this for sensor readings and hands it motor
// commands, exactly as it would the real hardware (HardwareIo). Behind the
// scenes, the QuadSim physics library (lib/QuadSim) turns those motor commands
// into motion, and the "sensor readings" are the simulated drone's true state.
// The real controller flies against real physics at the real 200 Hz rate.
//
// Frame mapping (LOCKED -- validated on the laptop in
// lib/QuadSim/examples/nav_check.cpp):
//   reported roll  = +quadsim_roll        (firmware +roll = banked right)
//   reported pitch = -quadsim_pitch       (firmware +pitch = nose up)
//   reported north = -quadsim_pos[0]      (QuadSim body-Forward faces world +x, so
//   reported east  = -quadsim_pos[1]       the nav axes are swapped AND flipped)
//   yaw is NOT free-flying: the sim turns the nose itself, toward the current
//   target at up to YAW_RATE_DPS (standing in for a yaw loop that works), and
//   reports that TRUE heading on the compass plus the true turn rate.
//   QuadSim yaw = +heading (validated on the laptop: every heading converges).
//
// It also plays the scenarios (see simulate/scenarios_esp): fakes the START
// switch, injects faults, and logs the flight to /flight.csv on the drone's
// flash; the DUMPLOG command streams it back.
// ============================================================================

#include <Arduino.h>
#include <LittleFS.h>
#include <math.h>
#include <QuadSim.h>                                // the on-chip physics (lib/QuadSim)
#include "../../flight/FlightController.hpp"   // fc:: API (start, land, phase, ...)
#include "../../flight/state/FlightSettings.hpp"
#include "../../flight/services/NavMath.hpp"        // gpsBearing, getMissionWaypoint
#include "../../flight/services/Log.hpp"            // logLine

class SimIo : public FlightIo {
public:
  static constexpr float  FT_PER_M      = 3.28084f;
  static constexpr double M_PER_DEG_LAT = 111320.0;
  static constexpr float  YAW_RATE_DPS  = 90.0f;     // how fast the sim turns the nose
  static constexpr float  CELL_RESISTANCE_OHM = 0.012f;   // battery sag per cell = amps × this

  String scenario = "full";   // chosen by the app before fc::begin()
  float  startCharge = 1.0f;  // true charge of the fake pack at power-up (lowbatt: 0.25)

  // --- FlightIo --------------------------------------------------------------

  void begin() override {
    // Home = the end of the route (routes end back at the launch point).
    homeLat_ = settings().mission.route.back().lat;
    homeLon_ = settings().mission.route.back().lon;
    lat_ = homeLat_;
    lon_ = homeLon_;
    charge_ = startCharge;
    t0Ms_ = millis();

    logFile_ = LittleFS.open(LOG_PATH, "w");
    if (logFile_) {
      logFile_.printf("# home_lat=%.6f home_lon=%.6f\n", homeLat_, homeLon_);
      logFile_.println("t_s,phase,north_m,east_m,up_ft,roll_deg,pitch_deg,dist_m,wp,cell_v,mah_est");
      logFile_.flush();
    }
    logLine("[SIM] on-chip physics ready -- scenario '" + scenario + "'.");
  }

  // 200 Hz: step the physics with the last motor command, report the result.
  void readFast(float dt, FastInputs& in) override {
    if (dt <= 0.0f || dt > 0.5f) dt = 0.005f;
    float m[4] = { mix_.m1, mix_.m2, mix_.m3, mix_.m4 };
    uav_.step(m, dt);

    // Drain the fake LiFe pack with the same current model the firmware
    // estimates with, and work out the voltage it shows (sagging under load).
    const FlightSettings& s = settings();
    float amps = s.battery.currentIdleA + s.battery.currentMotorFullA *
                 (m[0]*m[0]*m[0] + m[1]*m[1]*m[1] + m[2]*m[2]*m[2] + m[3]*m[3]*m[3]);
    charge_ -= amps * dt / 3.6f / s.battery.capacityMah;
    if (charge_ < 0.0f) charge_ = 0.0f;
    in.packVolts = s.battery.cells * (lifeRestCellVolts(charge_) - amps * CELL_RESISTANCE_OHM);

    // Turn the nose toward the goal heading, the short way round, no faster
    // than YAW_RATE_DPS.
    float turn = yawGoalDeg_ - yawDeg_;
    if (turn > 180.0f)  turn -= 360.0f;
    if (turn < -180.0f) turn += 360.0f;
    float maxTurn = YAW_RATE_DPS * dt;
    turn = constrain(turn, -maxTurn, maxTurn);
    yawDeg_ = fmodf(yawDeg_ + turn + 360.0f, 360.0f);
    float yawRate = turn / dt;   // deg/s, + = heading increasing

    // Pin yaw to that heading: rebuild the quaternion from roll, pitch and our
    // heading, zero the physics' own yaw rate. The stabilization scenario holds
    // a ~22 deg roll (a gust) for a moment, with body rates zeroed so it
    // recovers cleanly from 22 deg on release.
    float qr, qp, qy; uav_.rpyDeg(qr, qp, qy);
    bool kicking = millis() < kickUntilMs_;
    if (kicking) qr = 22.0f;
    quadsim::State& st = uav_.mutableState();
    float hr = qr * DEG_TO_RAD * 0.5f, hp = qp * DEG_TO_RAD * 0.5f, hy = yawDeg_ * DEG_TO_RAD * 0.5f;
    float cr = cosf(hr), sr = sinf(hr), cp = cosf(hp), sp = sinf(hp), cy = cosf(hy), sy = sinf(hy);
    st.quat[0] = cr * cp * cy + sr * sp * sy;   // roll-pitch-yaw -> quaternion
    st.quat[1] = sr * cp * cy - cr * sp * sy;
    st.quat[2] = cr * sp * cy + sr * cp * sy;
    st.quat[3] = cr * cp * sy - sr * sp * cy;
    st.omega[2] = 0.0f;
    if (kicking) { st.omega[0] = 0.0f; st.omega[1] = 0.0f; }

    // The sensor readings (LOCKED frame mapping -- see header).
    in.imu.gyroX      = +qr;        // fused roll  (Imu.hpp naming quirk)
    in.imu.gyroY      = -qp;        // fused pitch
    in.imu.gyroZ      = yawDeg_;    // fused yaw
    in.imu.yawRateDps = yawRate;
    in.baroAltitudeFt = uav_.altitude() * FT_PER_M;

    float north = -st.pos[0];
    float east  = -st.pos[1];
    withMutex([&]() {   // shared with readSlow, which runs on the other core
      lat_ = homeLat_ + north / M_PER_DEG_LAT;
      lon_ = homeLon_ + east / (M_PER_DEG_LAT * cos(homeLat_ * DEG_TO_RAD));
      roll_ = in.imu.gyroX; pitch_ = in.imu.gyroY; upFt_ = in.baroAltitudeFt;
    });
  }

  // 10 Hz: play the scenario, aim the nose, log a sample, and report GPS + compass.
  void readSlow(SlowInputs& in) override {
    double lat, lon; float roll, pitch, upFt;
    withMutex([&]() { lat = lat_; lon = lon_; roll = roll_; pitch = pitch_; upFt = upFt_; });

    playScenario();
    aimNose(lat, lon);
    logSample(lat, lon, roll, pitch, upFt);

    // GPS-loss scenario: no fix, and the fix time stops advancing, so the
    // GPS-loss failsafe trips after safety.gpsLossAbortMs.
    in.gps.lat       = lat;
    in.gps.lon       = lon;
    in.gps.fix       = !gpsLost_;
    in.gps.sats      = gpsLost_ ? 0 : 10;
    in.gps.lastFixMs = millis();
    in.compassHeadingDeg = yawDeg_;
  }

  void writeMotors(const MotorMix& mix) override { mix_ = mix; }
  void writeMotor(int, float) override {}
  void stopMotors() override { mix_ = MotorMix{}; }
  void stopMotor(int) override {}

  // DUMPLOG: stream the recorded flight log back over USB between markers.
  void dumpLog() {
    if (logFile_) logFile_.flush();
    File f = LittleFS.open(LOG_PATH, "r");
    if (!f) { logLine("[SIM] no log to dump"); return; }
    Serial.println("===ONBOARD_LOG_START===");
    while (f.available()) Serial.write(f.read());
    Serial.println("===ONBOARD_LOG_END===");
    f.close();
  }

private:
  static constexpr const char* LOG_PATH = "/flight.csv";

  // A ~4:1 thrust/weight airframe, so hover sits near airframe.hoverThrottle (0.5).
  static quadsim::Params airframe() {
    quadsim::Params p = quadsim::Params::generic();
    p.maxRotorSpeed = sqrtf(4.0f * p.mass * quadsim::kGravity / (4.0f * p.kThrust));
    return p;
  }

  // LiFe resting voltage per cell vs. charge left: flat ~3.30-3.40 V for most of
  // the pack, then a cliff below 20% (see Lesson 4.1 of the battery course).
  static float lifeRestCellVolts(float charge) {
    if (charge >= 0.2f) return 3.30f + 0.10f * (charge - 0.2f) / 0.8f;
    if (charge <= 0.0f) return 2.50f;
    return 2.50f + 0.80f * (charge / 0.2f);
  }

  // The scenario's script: fake the START switch, then inject its one fault.
  void playScenario() {
    unsigned long now = millis();
    unsigned long t   = now - t0Ms_;
    FlightPhase phase = fc::phase();
    const bool missionScenario = (scenario == "full" || scenario == "geofence" || scenario == "timeout" ||
                                  scenario == "rcloss" || scenario == "lowbatt" || scenario == "testroute");

    // Autostart: PARKED -> RAISE, then (mission scenarios) HOLD -> MISSION.
    if (t > 2500 && now - lastStartMs_ > 1500) {
      if (phase == PHASE_PARKED || (phase == PHASE_HOLD && missionScenario)) {
        fc::start();
        lastStartMs_ = now;
      }
    }

    // How long we've been hovering in HOLD (the land scenario waits a moment,
    // like a pilot would, instead of flipping LAND the instant HOLD begins).
    if (phase == PHASE_HOLD) { if (holdSinceMs_ == 0) holdSinceMs_ = now; }
    else holdSinceMs_ = 0;

    // Faults that act once, on reaching HOLD.
    if (phase == PHASE_HOLD && !acted_ && t > 2500) {
      if (scenario == "gpsloss") {
        gpsLost_ = true; acted_ = true;
        logLine("[SCENARIO] GPS fix dropped");
      } else if (scenario == "stab") {
        kickUntilMs_ = now + 500; acted_ = true; actedMs_ = now;
        logLine("[SCENARIO] attitude kick applied");
      } else if (scenario == "land") {
        if (now - holdSinceMs_ < 2000) return;   // hover 2 s first, then flip LAND
        fc::land(); acted_ = true;
        logLine("[SCENARIO] LAND switch flipped");
      } else if (scenario == "manual") {
        fc::manualOn(); acted_ = true; actedMs_ = now;
        logLine("[SCENARIO] manual control on");
      }
    }

    // Radio loss: a fake radio that goes silent 10 s into the mission.
    if (scenario == "rcloss") {
      if (phase == PHASE_MISSION && !acted_) { acted_ = true; actedMs_ = now; }
      bool radioAlive = !acted_ || now - actedMs_ < 10000;
      if (radioAlive) fc::radioHeartbeat();
      else if (!done_) { done_ = true; logLine("[SCENARIO] radio link cut"); }
    }

    // Manual: fly forward on the sticks for 5 s, then stop.
    if (scenario == "manual" && acted_ && !done_) {
      unsigned long e = now - actedMs_;
      if (phase == PHASE_MANUAL && e < 5000) {
        fc::setSticks(0.5f, 0.0f, 0.5f, 0.0f);
      } else if (e >= 5000) {
        fc::setSticks(0.5f, 0.0f, 0.0f, 0.0f);
        fc::stop();
        done_ = true;
        logLine("[SCENARIO] ===SCENARIO_DONE===");
      }
    }

    // Stabilization: after the kick, give it time to recover, then stop.
    if (scenario == "stab" && acted_ && !done_ && now - actedMs_ > 6000) {
      fc::stop();
      done_ = true;
      logLine("[SCENARIO] ===SCENARIO_DONE===");
    }
  }

  // Turn the nose toward the current target (readFast does the turning).
  void aimNose(double lat, double lon) {
    FlightPhase phase = fc::phase();
    double launchLat, launchLon;
    fc::launchPoint(launchLat, launchLon);
    if (phase == PHASE_MISSION) {
      Waypoint w = getMissionWaypoint(fc::missionWaypointIndex(), launchLat, launchLon);
      yawGoalDeg_ = gpsBearing(lat, lon, w.lat, w.lon);
    } else if (phase == PHASE_RTL_RETURN) {
      yawGoalDeg_ = gpsBearing(lat, lon, launchLat, launchLon);
    }
  }

  // One CSV row per nav tick (10 Hz).
  void logSample(double lat, double lon, float roll, float pitch, float upFt) {
    if (!logFile_) return;
    FlightPhase phase = fc::phase();
    RawBattery  b     = fc::batteryState();
    float north = (lat - homeLat_) * M_PER_DEG_LAT;
    float east  = (lon - homeLon_) * (M_PER_DEG_LAT * cos(homeLat_ * DEG_TO_RAD));
    float dist  = sqrtf(north * north + east * east);
    logFile_.printf("%.2f,%s,%.1f,%.1f,%.1f,%.2f,%.2f,%.1f,%d,%.3f,%.0f\n",
                    (millis() - t0Ms_) / 1000.0f, phaseName(phase), north, east, upFt,
                    roll, pitch, dist, fc::missionWaypointIndex(), b.cellVolts, b.mAhUsed);
    if (phase == PHASE_LANDED) logFile_.flush();
  }

  quadsim::Multirotor uav_{airframe(), quadsim::State::level(0.0f)};
  MotorMix      mix_{};
  File          logFile_;
  double        homeLat_ = 0, homeLon_ = 0, lat_ = 0, lon_ = 0;
  float         roll_ = 0, pitch_ = 0, upFt_ = 0;
  float         yawDeg_ = 0, yawGoalDeg_ = 0;
  float         charge_ = 1.0f;
  unsigned long t0Ms_ = 0, lastStartMs_ = 0, kickUntilMs_ = 0, actedMs_ = 0, holdSinceMs_ = 0;
  bool          gpsLost_ = false, acted_ = false, done_ = false;
};
