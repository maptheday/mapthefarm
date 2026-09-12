#pragma once

// Result of translating a control decision into four motor commands.
struct MotorMix {
  float m1;
  float m2;
  float m3;
  float m4;
  float baseThrottle;
  float rollCorrection;
  float pitchCorrection;
};