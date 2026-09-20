// QuadSim desktop example / smoke test.
//   c++ -std=c++17 -I ../src hover.cpp -o hover && ./hover
// Because QuadSim is pure C++, the exact model that runs on the MCU runs here.
#include "QuadSim.h"
#include <cstdio>

int main() {
  using namespace quadsim;
  Params p = Params::generic();
  float hover = std::sqrt(p.mass * kGravity / (4.0f * p.kThrust)) / p.maxRotorSpeed;
  std::printf("hover throttle = %.3f  (max rotor %.0f rad/s)\n", hover, p.maxRotorSpeed);

  // 1) Hover: all four at hover throttle -> altitude should hold near start.
  {
    Multirotor uav(p, State::level(5.0f));
    float m[4] = {hover, hover, hover, hover};
    for (int i = 0; i < 300; ++i) uav.step(m, 0.01f);   // 3 s
    float r, pt, y; uav.rpyDeg(r, pt, y);
    std::printf("HOVER 3s:  alt=%.2f m (start 5.0)  roll=%.1f pitch=%.1f\n",
                uav.altitude(), r, pt);
  }
  // 2) Left rotors harder (FL,RL) -> should bank RIGHT (+roll).
  {
    Multirotor uav(p, State::level(5.0f));
    float m[4] = {hover * 1.1f, hover * 0.9f, hover * 1.1f, hover * 0.9f}; // FL,FR,RL,RR
    for (int i = 0; i < 20; ++i) uav.step(m, 0.01f);   // 0.2 s
    float r, pt, y; uav.rpyDeg(r, pt, y);
    std::printf("LEFT>RIGHT 0.2s:  roll=%+.1f (expect +, banks right)  pitch=%+.1f\n", r, pt);
  }
  // 3) Rear rotors harder (RL,RR) -> pitches; report the sign.
  {
    Multirotor uav(p, State::level(5.0f));
    float m[4] = {hover * 0.9f, hover * 0.9f, hover * 1.1f, hover * 1.1f};
    for (int i = 0; i < 20; ++i) uav.step(m, 0.01f);
    float r, pt, y; uav.rpyDeg(r, pt, y);
    std::printf("REAR>FRONT 0.2s:  pitch=%+.1f  roll=%+.1f\n", pt, r);
  }
  // 4) Diagonal (FL,RR) harder -> yaw reaction should spin it.
  {
    Multirotor uav(p, State::level(5.0f));
    float m[4] = {hover * 1.1f, hover * 0.9f, hover * 0.9f, hover * 1.1f};
    for (int i = 0; i < 50; ++i) uav.step(m, 0.01f);
    float r, pt, y; uav.rpyDeg(r, pt, y);
    std::printf("DIAG FL,RR 0.5s:  yaw=%+.1f (expect non-zero)\n", y);
  }
  return 0;
}
