#pragma once
#include "../interfaces/IESC.hpp"
#include <algorithm>

// ============================================================
//  MotorMixer
//
//  Combines throttle, roll, pitch, and yaw commands into
//  four individual motor outputs.
//
//  Motor layout (top-down view):
//
//           FRONT
//      M1(CCW)  M2(CW)
//         \      /
//          \    /
//          /    \
//         /      \
//      M3(CW)  M4(CCW)
//           REAR
//
//  How each axis works:
//
//    THROTTLE  — all four motors increase/decrease equally
//
//    ROLL RIGHT — left motors (M1, M3) speed up
//                 right motors (M2, M4) slow down
//
//    PITCH FWD  — rear motors (M3, M4) speed up
//                 front motors (M1, M2) slow down
//
//    YAW RIGHT  — CCW motors (M1, M4) speed up
//                 CW  motors (M2, M3) slow down
//
//  Yaw works because of motor spin direction. Each motor
//  produces a small torque reaction in the opposite direction
//  of its spin. CCW motors push the frame clockwise (yaw right)
//  and CW motors push the frame counter-clockwise (yaw left).
//  Speeding up one diagonal pair overpowers the other and
//  rotates the whole drone around its yaw axis.
// ============================================================
class MotorMixer {
public:
    explicit MotorMixer(IESC& esc) : esc(esc) {}

    // Call every Core 1 cycle when armed.
    //
    //  throttle  — 0.0 to 1.0
    //  roll_cmd  — negative = roll left,    positive = roll right
    //  pitch_cmd — negative = pitch forward, positive = pitch back
    //  yaw_cmd   — negative = yaw left,     positive = yaw right
    //
    void mix(double throttle, double roll_cmd, double pitch_cmd, double yaw_cmd) {
        double m1 = throttle - roll_cmd - pitch_cmd + yaw_cmd;  // front-left  (CCW)
        double m2 = throttle + roll_cmd - pitch_cmd - yaw_cmd;  // front-right (CW)
        double m3 = throttle - roll_cmd + pitch_cmd - yaw_cmd;  // rear-left   (CW)
        double m4 = throttle + roll_cmd + pitch_cmd + yaw_cmd;  // rear-right  (CCW)

        esc.write(clamp(m1), clamp(m2), clamp(m3), clamp(m4));
    }

    void disarm() {
        esc.write(0.0, 0.0, 0.0, 0.0);
    }

private:
    IESC& esc;

    static double clamp(double v) {
        return std::max(0.0, std::min(1.0, v));
    }
};