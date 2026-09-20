// ============================================================================
// QuadSim -- a tiny quadrotor physics simulator for on-chip software-in-the-loop.
//
// Pure C++ (only <cmath>): no Arduino, no ESP, no heap. The SAME model compiles
// and runs on a microcontroller and on a desktop, so you can close your flight
// controller's loop against real physics at the real control rate -- no laptop
// in the loop, no serial latency, one clock.
//
// Conventions (documented so integrators can map their own frames):
//   World frame : ENU  (x=East, y=North, z=Up). Gravity pulls -z.
//   Body frame  : FLU  (x=Forward, y=Left, z=Up). Thrust is +z (up) in body.
//   Rotors      : X-config, 4 motors, indexed [0..3] = FL, FR, RL, RR.
//                 (front-left, front-right, rear-left, rear-right)
//                 spin dirs [+, -, -, +] -> diagonals spin together.
//   Motor input : normalized throttle 0..1 per rotor (0 = stopped, 1 = max).
//   Attitude    : unit quaternion (w,x,y,z), body->world. rpyDeg() returns the
//                 aerospace 3-2-1 roll(x)/pitch(y)/yaw(z) in degrees.
//
// A right-hand rule sanity check with these conventions:
//   * more thrust on the LEFT rotors  -> banks RIGHT (+roll)
//   * more thrust on the REAR rotors  -> pitches nose UP is NEGATIVE pitch here;
//     see rpyDeg() -- integrators pick the sign their firmware expects.
//
// Usage:
//   quadsim::Multirotor uav(quadsim::Params::generic());
//   uav.step(motor /*float[4], 0..1*/, dt /*seconds*/);
//   float r,p,y; uav.rpyDeg(r,p,y);  float altM = uav.altitude();
// ============================================================================
#pragma once
#include <cmath>

namespace quadsim {

constexpr float kGravity = 9.81f;      // m/s^2
constexpr float kSubStep = 0.001f;     // internal integration step (s)

// ---- Physical parameters of one airframe -----------------------------------
struct Params {
  float mass;           // kg
  float Ixx, Iyy, Izz;  // body moments of inertia (kg*m^2)
  float armLength;      // motor-to-center distance (m)
  float kThrust;        // thrust per rotor = kThrust * speed^2   (N per (rad/s)^2)
  float kTorque;        // yaw reaction  = kTorque * speed^2       (N*m per (rad/s)^2)
  float maxRotorSpeed;  // rotor speed at throttle 1.0 (rad/s)
  float dragXY;         // linear air drag, horizontal (N per m/s)
  float dragZ;          // linear air drag, vertical   (N per m/s)

  // A sensible generic ~1.2 kg quad with ~2:1 thrust/weight and gentle drag.
  static Params generic() {
    Params p;
    p.mass = 1.2f;
    p.Ixx = 0.015f; p.Iyy = 0.015f; p.Izz = 0.026f;
    p.armLength = 0.23f;
    p.kThrust = 8.0e-6f;
    p.kTorque = 1.2e-7f;
    // hover: 4*kThrust*w_h^2 = m*g  ->  full throttle ~2.1:1 thrust/weight
    p.maxRotorSpeed = std::sqrt(2.1f * p.mass * kGravity / (4.0f * p.kThrust));
    p.dragXY = 0.30f; p.dragZ = 0.40f;
    return p;
  }
};

// ---- Full rigid-body state --------------------------------------------------
struct State {
  float pos[3];    // world ENU position (m)
  float vel[3];    // world velocity (m/s)
  float quat[4];   // orientation body->world, (w,x,y,z), unit
  float omega[3];  // body angular rates (rad/s)

  // Level, at rest, at altitude altM.
  static State level(float altM) {
    State s{};
    s.pos[0] = s.pos[1] = 0.0f; s.pos[2] = altM;
    s.vel[0] = s.vel[1] = s.vel[2] = 0.0f;
    s.quat[0] = 1.0f; s.quat[1] = s.quat[2] = s.quat[3] = 0.0f;
    s.omega[0] = s.omega[1] = s.omega[2] = 0.0f;
    return s;
  }
};

// ---- The simulator ----------------------------------------------------------
class Multirotor {
public:
  bool restOnGround = true;   // stop falling through z=0

  explicit Multirotor(const Params& params, const State& initial = State::level(0.0f))
    : p_(params), s_(initial) {}

  // Advance the physics by dt seconds given four normalized throttles (FL,FR,RL,RR).
  // Internally sub-steps so a large dt stays stable.
  void step(const float motor[4], float dt) {
    if (dt <= 0.0f) return;
    int n = (int)std::ceil(dt / kSubStep);
    if (n < 1) n = 1;
    if (n > 4000) n = 4000;           // guard against absurd dt
    float h = dt / (float)n;
    for (int i = 0; i < n; ++i) integrate(motor, h);
  }

  const State& state() const { return s_; }
  const Params& params() const { return p_; }
  State& mutableState() { return s_; }   // for injecting disturbances / resets

  float altitude() const { return s_.pos[2]; }

  // Roll (x), pitch (y), yaw (z) in DEGREES, aerospace body 3-2-1.
  void rpyDeg(float& roll, float& pitch, float& yaw) const {
    const float w = s_.quat[0], x = s_.quat[1], y = s_.quat[2], z = s_.quat[3];
    roll  = std::atan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
    float sp = 2.0f * (w * y - z * x);
    sp = sp > 1.0f ? 1.0f : (sp < -1.0f ? -1.0f : sp);
    pitch = std::asin(sp);
    yaw   = std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
    const float r2d = 57.29577951f;
    roll *= r2d; pitch *= r2d; yaw *= r2d;
  }

  // Yaw rate about body z, in deg/s (handy for a gyro-Z feed).
  float yawRateDeg() const { return s_.omega[2] * 57.29577951f; }

private:
  Params p_;
  State  s_;

  void integrate(const float motor[4], float h) {
    // rotor speeds and per-rotor thrust (N), order FL,FR,RL,RR
    float T[4];
    for (int i = 0; i < 4; ++i) {
      float c = motor[i] < 0.0f ? 0.0f : (motor[i] > 1.0f ? 1.0f : motor[i]);
      float w = c * p_.maxRotorSpeed;
      T[i] = p_.kThrust * w * w;
    }
    float thrust = T[0] + T[1] + T[2] + T[3];   // total, along body +z

    // --- linear dynamics (world frame) ---
    // rotate body thrust (0,0,thrust) into world by the quaternion
    float tw[3]; rotateBodyToWorld(0.0f, 0.0f, thrust, tw);
    float ax = (tw[0] - p_.dragXY * s_.vel[0]) / p_.mass;
    float ay = (tw[1] - p_.dragXY * s_.vel[1]) / p_.mass;
    float az = (tw[2] - p_.dragZ  * s_.vel[2]) / p_.mass - kGravity;
    s_.vel[0] += ax * h; s_.vel[1] += ay * h; s_.vel[2] += az * h;
    s_.pos[0] += s_.vel[0] * h; s_.pos[1] += s_.vel[1] * h; s_.pos[2] += s_.vel[2] * h;

    if (restOnGround && s_.pos[2] <= 0.0f) {
      s_.pos[2] = 0.0f;
      if (s_.vel[2] < 0.0f) s_.vel[2] = 0.0f;
      s_.vel[0] *= 0.85f; s_.vel[1] *= 0.85f;   // ground friction
    }

    // --- rotational dynamics (body frame) ---
    const float d = p_.armLength * 0.70710678f;  // X-config lever arm on each axis
    // roll about +x: left rotors (FL,RL) minus right (FR,RR)
    float tauX = d * ((T[0] + T[2]) - (T[1] + T[3]));
    // pitch about +y: rear rotors (RL,RR) minus front (FL,FR)
    float tauY = d * ((T[2] + T[3]) - (T[0] + T[1]));
    // yaw about +z: reaction torque, spin dirs [+,-,-,+]
    float wsq[4];
    for (int i = 0; i < 4; ++i) {
      float c = motor[i] < 0.0f ? 0.0f : (motor[i] > 1.0f ? 1.0f : motor[i]);
      float w = c * p_.maxRotorSpeed; wsq[i] = w * w;
    }
    float tauZ = p_.kTorque * (wsq[0] - wsq[1] - wsq[2] + wsq[3]);

    // Euler's rigid body eqns: I*wdot = tau - w x (I*w)
    float Ix = p_.Ixx, Iy = p_.Iyy, Iz = p_.Izz;
    float wx = s_.omega[0], wy = s_.omega[1], wz = s_.omega[2];
    float gyroX = wy * wz * (Iz - Iy);
    float gyroY = wz * wx * (Ix - Iz);
    float gyroZ = wx * wy * (Iy - Ix);
    float wdotX = (tauX - gyroX) / Ix;
    float wdotY = (tauY - gyroY) / Iy;
    float wdotZ = (tauZ - gyroZ) / Iz;
    s_.omega[0] += wdotX * h; s_.omega[1] += wdotY * h; s_.omega[2] += wdotZ * h;

    // integrate quaternion: qdot = 0.5 * q (x) (0, omega)
    float w0 = s_.quat[0], x0 = s_.quat[1], y0 = s_.quat[2], z0 = s_.quat[3];
    float ox = s_.omega[0], oy = s_.omega[1], oz = s_.omega[2];
    float dw = 0.5f * (-x0 * ox - y0 * oy - z0 * oz);
    float dx = 0.5f * ( w0 * ox + y0 * oz - z0 * oy);
    float dy = 0.5f * ( w0 * oy - x0 * oz + z0 * ox);
    float dz = 0.5f * ( w0 * oz + x0 * oy - y0 * ox);
    w0 += dw * h; x0 += dx * h; y0 += dy * h; z0 += dz * h;
    float nrm = std::sqrt(w0 * w0 + x0 * x0 + y0 * y0 + z0 * z0);
    if (nrm < 1e-6f) { w0 = 1; x0 = y0 = z0 = 0; nrm = 1; }
    s_.quat[0] = w0 / nrm; s_.quat[1] = x0 / nrm; s_.quat[2] = y0 / nrm; s_.quat[3] = z0 / nrm;
  }

  // v_world = q * v_body * q^-1
  void rotateBodyToWorld(float vx, float vy, float vz, float out[3]) const {
    const float w = s_.quat[0], x = s_.quat[1], y = s_.quat[2], z = s_.quat[3];
    // t = 2 * (q_vec x v)
    float tx = 2.0f * (y * vz - z * vy);
    float ty = 2.0f * (z * vx - x * vz);
    float tz = 2.0f * (x * vy - y * vx);
    out[0] = vx + w * tx + (y * tz - z * ty);
    out[1] = vy + w * ty + (z * tx - x * tz);
    out[2] = vz + w * tz + (x * ty - y * tx);
  }
};

}  // namespace quadsim
