#pragma once

// ============================================================================
// SIM WORLD -- what is TRUE in the simulation: where the drone really is, how
// it's really tilted, how much charge the pack really has.
//
// It runs on its own clock (its own 200 Hz loop), like the real world does:
// the drone's motors push it, and the drone's sensors look at it, but it
// carries on whether anyone is looking or not.
//
//   motors  ──► setMotors(mix) ──► QuadSim physics step (every 5 ms)
//                                        │
//   sensors ◄──────── truth() ◄──────────┘   position, tilt, heading, pack voltage
//
// The QuadSim library (lib/QuadSim) does the physics: motor thrust -> force ->
// acceleration -> velocity -> position. truth() hands the result back already
// in the flight controller's conventions.
//
// Frame mapping (LOCKED -- validated on the laptop in
// lib/QuadSim/examples/nav_check.cpp):
//   roll  = +quadsim_roll        (firmware +roll = banked right)
//   pitch = -quadsim_pitch       (firmware +pitch = nose up)
//   north = -quadsim_pos[0]      (QuadSim body-Forward faces world +x, so
//   east  = -quadsim_pos[1]       the nav axes are swapped AND flipped)
// Yaw is NOT free-flying: the world turns the nose itself, toward the current
// target at up to YAW_RATE_DPS (standing in for a yaw loop that works), and
// that is the true heading and turn rate. QuadSim yaw = +heading.
//
// Scenarios change the world through gust() and setCharge(), never by
// changing what a sensor says (that's a sensor wrapper's job, see scenarios/).
// ============================================================================

#include <Arduino.h>
#include <math.h>
#include <QuadSim.h>                          // the on-chip physics (lib/QuadSim)
#include "flight/FlightController.hpp"        // fc::phase(), fc::launchPoint(), ...
#include "flight/services/NavMath.hpp"        // gpsBearing, getMissionWaypoint

// The true state, in the flight controller's conventions.
struct Truth {
  float  rollDeg = 0, pitchDeg = 0;      // firmware signs (+roll = right, +pitch = nose up)
  float  headingDeg = 0, yawRateDps = 0; // clockwise from north
  float  upFt = 0;                       // height above the launch point
  float  northM = 0, eastM = 0;          // from home
  double lat = 0, lon = 0;
  float  packVolts = 0;
};

class SimWorld {
public:
  static constexpr float  FT_PER_M      = 3.28084f;
  static constexpr double M_PER_DEG_LAT = 111320.0;
  static constexpr float  YAW_RATE_DPS  = 90.0f;          // how fast the world turns the nose
  static constexpr float  CELL_RESISTANCE_OHM = 0.012f;   // battery sag per cell = amps × this
  static constexpr float  STEP_S        = 0.005f;         // 200 Hz, like the physics loop

  // Where the drone is at power-up: the end of the mission route (routes end
  // back at the launch point). Call after the settings are loaded.
  void begin() {
    lock_ = xSemaphoreCreateMutex();
    homeLat_ = settings().mission.route.back().lat;
    homeLon_ = settings().mission.route.back().lon;
    update(0.0f);   // a first truth, before the clock starts
    xTaskCreatePinnedToCore(worldTask, "SimWorld", 8192, this, 3, NULL, 0);
    logLine("[SIM] world running -- the physics has its own 200 Hz clock.");
  }

  // --- what the sensors see -------------------------------------------------
  Truth  truth()  { Truth t; locked([&]() { t = truth_; }); return t; }
  double homeLat() const { return homeLat_; }
  double homeLon() const { return homeLon_; }

  // --- what the motors do ---------------------------------------------------
  void setMotors(const MotorMix& mix) { locked([&]() { mix_ = mix; }); }

  // --- what a scenario can do to the world ----------------------------------

  // A gust: hold the drone rolled over by `rollDeg` for `ms`, then let go.
  void gust(float rollDeg, unsigned long ms) {
    locked([&]() { gustRollDeg_ = rollDeg; gustUntilMs_ = millis() + ms; });
  }

  // How charged the pack really is, 0..1 (1 = full). Before begin().
  void setCharge(float charge) { charge_ = charge; }

private:
  static void worldTask(void* self) {
    SimWorld& w = *(SimWorld*)self;
    TickType_t lastWake = xTaskGetTickCount();
    for (int n = 0;; n++) {
      w.update(STEP_S);
      if (n % 20 == 0) w.aimNose();   // 10 Hz
      vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(5));
    }
  }

  // One physics step of `dt` seconds, then work out the new truth.
  void update(float dt) {
    locked([&]() {
      float m[4] = { mix_.m1, mix_.m2, mix_.m3, mix_.m4 };
      if (dt > 0.0f) uav_.step(m, dt);

      // Drain the pack with the same current model the firmware estimates with,
      // and work out the voltage it shows (sagging under load).
      const FlightSettings& s = settings();
      float amps = s.battery.currentIdleA + s.battery.currentMotorFullA *
                   (m[0]*m[0]*m[0] + m[1]*m[1]*m[1] + m[2]*m[2]*m[2] + m[3]*m[3]*m[3]);
      charge_ -= amps * dt / 3.6f / s.battery.capacityMah;
      if (charge_ < 0.0f) charge_ = 0.0f;

      // Turn the nose toward the goal heading, the short way round, no faster
      // than YAW_RATE_DPS.
      float yawRate = 0.0f;
      if (dt > 0.0f) {
        float turn = yawGoalDeg_ - yawDeg_;
        if (turn > 180.0f)  turn -= 360.0f;
        if (turn < -180.0f) turn += 360.0f;
        float maxTurn = YAW_RATE_DPS * dt;
        turn = constrain(turn, -maxTurn, maxTurn);
        yawDeg_ = fmodf(yawDeg_ + turn + 360.0f, 360.0f);
        yawRate = turn / dt;   // deg/s, + = heading increasing
      }

      // Pin yaw to that heading: rebuild the quaternion from roll, pitch and
      // our heading, zero the physics' own yaw rate. A gust holds the roll
      // over for a moment, with body rates zeroed so it recovers cleanly on
      // release.
      float qr, qp, qy; uav_.rpyDeg(qr, qp, qy);
      bool gusting = millis() < gustUntilMs_;
      if (gusting) qr = gustRollDeg_;
      quadsim::State& st = uav_.mutableState();
      float hr = qr * DEG_TO_RAD * 0.5f, hp = qp * DEG_TO_RAD * 0.5f, hy = yawDeg_ * DEG_TO_RAD * 0.5f;
      float cr = cosf(hr), sr = sinf(hr), cp = cosf(hp), sp = sinf(hp), cy = cosf(hy), sy = sinf(hy);
      st.quat[0] = cr * cp * cy + sr * sp * sy;   // roll-pitch-yaw -> quaternion
      st.quat[1] = sr * cp * cy - cr * sp * sy;
      st.quat[2] = cr * sp * cy + sr * cp * sy;
      st.quat[3] = cr * cp * sy - sr * sp * cy;
      st.omega[2] = 0.0f;
      if (gusting) { st.omega[0] = 0.0f; st.omega[1] = 0.0f; }

      // The truth, in firmware conventions (LOCKED frame mapping -- see header).
      truth_.rollDeg    = +qr;
      truth_.pitchDeg   = -qp;
      truth_.headingDeg = yawDeg_;
      truth_.yawRateDps = yawRate;
      truth_.upFt       = uav_.altitude() * FT_PER_M;
      truth_.northM     = -st.pos[0];
      truth_.eastM      = -st.pos[1];
      truth_.lat        = homeLat_ + truth_.northM / M_PER_DEG_LAT;
      truth_.lon        = homeLon_ + truth_.eastM / (M_PER_DEG_LAT * cos(homeLat_ * DEG_TO_RAD));
      truth_.packVolts  = s.battery.cells * (lifeRestCellVolts(charge_) - amps * CELL_RESISTANCE_OHM);
    });
  }

  // Point the nose at whatever the drone is flying to (the stand-in for a
  // working yaw loop -- see header). Reads the flight controller's state, so
  // it runs outside the world lock.
  void aimNose() {
    Truth t = truth();
    FlightPhase phase = fc::phase();
    double launchLat, launchLon;
    fc::launchPoint(launchLat, launchLon);
    float goal;
    if (phase == PHASE_MISSION) {
      Waypoint w = getMissionWaypoint(fc::missionWaypointIndex(), launchLat, launchLon);
      goal = gpsBearing(t.lat, t.lon, w.lat, w.lon);
    } else if (phase == PHASE_RTL_RETURN) {
      goal = gpsBearing(t.lat, t.lon, launchLat, launchLon);
    } else {
      return;
    }
    locked([&]() { yawGoalDeg_ = goal; });
  }

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

  // The world's own lock (not the flight controller's "pen"): the world loop,
  // the drone's sensors and motors, and the scenario all reach in here.
  template <typename Fn> void locked(Fn fn) {
    xSemaphoreTake(lock_, portMAX_DELAY);
    fn();
    xSemaphoreGive(lock_);
  }

  SemaphoreHandle_t   lock_ = nullptr;
  quadsim::Multirotor uav_{airframe(), quadsim::State::level(0.0f)};
  MotorMix      mix_{};
  Truth         truth_{};
  double        homeLat_ = 0, homeLon_ = 0;
  float         yawDeg_ = 0, yawGoalDeg_ = 0;
  float         charge_ = 1.0f;
  float         gustRollDeg_ = 0;
  unsigned long gustUntilMs_ = 0;
};
