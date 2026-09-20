// QuadSim NAV/SIGN calibration harness (laptop only).
//   c++ -std=c++17 -I ../src nav_check.cpp -o nav_check && ./nav_check
//
// Purpose: lock the frame-mapping signs for the ON-CHIP sim BEFORE flashing.
// It replicates the firmware's real control math (PID.hpp, MotorController's
// computeMotorMix, MissionPhase's flat north->pitch / east->roll nav) and flies
// it against QuadSim, then reports which GPS position-feedback signs make the
// closed loop actually converge on a waypoint.
//
// Locked by attitude STABILITY (derived, see notes in OnboardSim.hpp):
//   reported roll  = +quadsim_roll     (ROLL_SIGN  = +1)
//   reported pitch = -quadsim_pitch    (PITCH_SIGN = -1)
// Free (what this harness solves): how QuadSim world position maps to the
//   reported GPS north/east the nav loop chases.
#include "QuadSim.h"
#include <cstdio>
#include <cmath>
#include <initializer_list>

using namespace quadsim;

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// ---- a faithful copy of the firmware PID (src/services/PID.hpp) ----
struct Pid {
  float kp, ki, kd, lo, hi, integ = 0, last = 0;
  Pid(float p, float i, float d, float mn, float mx) : kp(p), ki(i), kd(d), lo(mn), hi(mx) {}
  void reset() { integ = 0; last = 0; }
  float withError(float error, float dt) {
    if (dt <= 0) return 0;
    float p = kp * error;
    integ += error * dt;
    if (ki != 0) integ = clampf(integ, lo / ki, hi / ki);
    float i = ki * integ;
    float d = kd * (error - last) / dt;
    last = error;
    return clampf(p + i + d, lo, hi);
  }
  float compute(float target, float actual, float dt) { return withError(target - actual, dt); }
};

// ---- firmware constants (FlightConfig.hpp / MotorController.hpp) ----
constexpr float HOVER_THROTTLE_FF = 0.50f;
constexpr float DEG2RAD = 0.0174532925f;

// ---- a faithful copy of MotorController::computeMotorMix mixing ----
struct Controller {
  Pid altitude{0.02f, 0.008f, 0.10f, -0.45f, 0.45f};
  Pid roll{0.01f, 0.001f, 0.005f, -0.3f, 0.3f};
  Pid pitch{0.01f, 0.001f, 0.005f, -0.3f, 0.3f};
  void mix(float targetAltFt, float targetRollDeg, float targetPitchDeg,
           float altFt, float rollDeg, float pitchDeg, float dt, float m[4]) {
    float base = HOVER_THROTTLE_FF + altitude.compute(targetAltFt, altFt, dt);
    float tilt = 1.0f / (std::cos(rollDeg * DEG2RAD) * std::cos(pitchDeg * DEG2RAD));
    tilt = clampf(tilt, 1.0f, 2.0f);
    base = clampf(base * tilt, 0.0f, 1.0f);
    float rc = roll.compute(targetRollDeg, rollDeg, dt);
    float pc = pitch.compute(targetPitchDeg, pitchDeg, dt);
    // yaw held quiet (compass == target); m1..m4 order = FL,FR,RL,RR
    m[0] = clampf(base + pc + rc, 0.0f, 1.0f);
    m[1] = clampf(base + pc - rc, 0.0f, 1.0f);
    m[2] = clampf(base - pc + rc, 0.0f, 1.0f);
    m[3] = clampf(base - pc - rc, 0.0f, 1.0f);
  }
};

// A ~4:1 thrust/weight airframe so hover sits near HOVER_THROTTLE_FF (0.5),
// matching what the firmware's feed-forward assumes.
static Params airframe(float dragXY) {
  Params p = Params::generic();
  p.maxRotorSpeed = std::sqrt(4.0f * p.mass * kGravity / (4.0f * p.kThrust)); // full ~4:1
  p.dragXY = dragXY;
  return p;
}

// Fly to a waypoint NORTH/EAST of the start; return closest approach (m).
static float fly(int northSign, int eastSign, float wpNorth, float wpEast, float dragXY, bool verbose) {
  Params p = airframe(dragXY);
  Multirotor uav(p, State::level(0.0f));
  Controller ctl;
  Pid navNorth(0.35f, 0.0f, 0.6f, -6.0f, 6.0f);
  Pid navEast(0.35f, 0.0f, 0.6f, -6.0f, 6.0f);

  const float targetAltFt = 30.0f;
  const float FT_PER_M = 3.28084f;
  const float physDt = 0.005f;         // 200 Hz
  const int navEvery = 20;             // -> 10 Hz nav
  float m[4] = {0, 0, 0, 0};
  float targetRoll = 0, targetPitch = 0;
  float best = 1e9f;

  for (int step = 0; step < 200 * 40; ++step) {   // up to 40 s
    // reported sensors (the frame mapping under test)
    float qr, qp, qy; uav.rpyDeg(qr, qp, qy);
    float reportedRoll = +qr;
    float reportedPitch = -qp;
    float reportedAltFt = uav.altitude() * FT_PER_M;
    // QuadSim identity orientation faces body-Forward along world +x, so PITCH
    // drives motion along world-x (pos[0]) and ROLL along world-y (pos[1]) --
    // i.e. the nav axes are SWAPPED vs the raw world axes. Map accordingly:
    float north = northSign * uav.state().pos[0];   // pitch axis  <- world x
    float east = eastSign * uav.state().pos[1];      // roll axis   <- world y

    if (step % navEvery == 0) {
      float navDt = physDt * navEvery;
      float northErr = wpNorth - north;   // vector to waypoint
      float eastErr = wpEast - east;
      targetRoll = navEast.withError(eastErr, navDt);    // east -> roll  (MissionPhase)
      targetPitch = navNorth.withError(northErr, navDt); // north -> pitch
    }
    ctl.mix(targetAltFt, targetRoll, targetPitch, reportedAltFt, reportedRoll, reportedPitch, physDt, m);
    uav.step(m, physDt);

    // pin yaw to 0 (drone flies world-aligned; firmware keeps yaw quiet per-leg)
    State& s = uav.mutableState();
    float qr2, qp2, qy2; uav.rpyDeg(qr2, qp2, qy2);
    float cr = std::cos(qr2 * DEG2RAD * 0.5f), sr = std::sin(qr2 * DEG2RAD * 0.5f);
    float cp = std::cos(qp2 * DEG2RAD * 0.5f), sp = std::sin(qp2 * DEG2RAD * 0.5f);
    // rebuild quaternion from roll,pitch only (yaw=0): q = qy(0)*qp*qr
    s.quat[0] = cr * cp; s.quat[1] = sr * cp; s.quat[2] = cr * sp; s.quat[3] = -sr * sp;
    s.omega[2] = 0.0f;

    float dist = std::hypot(wpNorth - north, wpEast - east);
    if (dist < best) best = dist;
    if (verbose && step % 400 == 0)
      std::printf("   t=%4.1fs N=%6.1f E=%6.1f alt=%4.0fft  distToWP=%5.1f\n",
                  step * physDt, north, east, reportedAltFt, dist);
    if (dist < 4.0f) break;   // WAYPOINT_ACCEPT_RADIUS_M (field demo)
  }
  return best;
}

int main() {
  const float wpN = 20.0f, wpE = 15.0f;   // waypoint 20 m north, 15 m east
  std::printf("Step 1: sweep GPS position-feedback signs (roll +, pitch - locked, drag=0.30).\n");
  std::printf("Waypoint at N=%.0f E=%.0f m; a converging sign set reaches <4 m.\n\n", wpN, wpE);
  int bestN = 0, bestE = 0; float bestDist = 1e9f;
  for (int ns : {+1, -1})
    for (int es : {+1, -1}) {
      float d = fly(ns, es, wpN, wpE, 0.30f, false);
      std::printf("  NORTH_SIGN=%+d EAST_SIGN=%+d  ->  closest %6.1f m %s\n",
                  ns, es, d, d < 4.0f ? "  CONVERGES" : "(heads there then diverges)");
      if (d < bestDist) { bestDist = d; bestN = ns; bestE = es; }
    }
  std::printf("\nDirection winner: NORTH_SIGN=%+d EAST_SIGN=%+d.\n", bestN, bestE);

  std::printf("\nStep 2: with those signs, sweep horizontal drag (N per m/s) for clean capture.\n");
  float bestDrag = 0; float bestDragDist = 1e9f;
  for (float dr : {0.30f, 0.45f, 0.60f, 0.75f, 0.90f, 1.10f}) {
    float d = fly(bestN, bestE, wpN, wpE, dr, false);
    std::printf("  dragXY=%.2f  ->  closest %6.1f m %s\n", dr, d, d < 4.0f ? "  CAPTURES <4 m" : "");
    if (d < bestDragDist) { bestDragDist = d; bestDrag = dr; }
  }
  std::printf("\nBest: signs (%+d,%+d), dragXY=%.2f -> closest %.1f m. Detailed run:\n",
              bestN, bestE, bestDrag, bestDragDist);
  fly(bestN, bestE, wpN, wpE, bestDrag, true);
  return 0;
}
