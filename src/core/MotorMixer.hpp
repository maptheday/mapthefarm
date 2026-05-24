#pragma once
#include "../interfaces/IESC.hpp"
#include <algorithm>

// ============================================================
//  MotorMixer
//
//  Takes the three control commands from the PID loops and
//  works out what speed each of the four motors needs to run
//  at to produce that combined motion.
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
//  How the mixing works:
//
//    THROTTLE  — all four motors increase/decrease equally
//    ROLL RIGHT — left motors (M1, M3) speed up
//                 right motors (M2, M4) slow down
//    PITCH FWD  — rear motors (M3, M4) speed up
//                 front motors (M1, M2) slow down
// ============================================================
class MotorMixer {
public:
    explicit MotorMixer(IESC& esc) : esc(esc) {}

    // Call this every Core 1 cycle when armed.
    //
    //  throttle  — 0.0 to 1.0  (from altitude PID + base hover)
    //  roll_cmd  — negative = roll left, positive = roll right
    //  pitch_cmd — negative = pitch forward, positive = pitch back
    //
    void mix(double throttle, double roll_cmd, double pitch_cmd) {

        // 1. Calculate raw ideal motor outputs
        double m1_raw = throttle - roll_cmd - pitch_cmd;  // front-left  (CCW)
        double m2_raw = throttle + roll_cmd - pitch_cmd;  // front-right (CW)
        double m3_raw = throttle - roll_cmd + pitch_cmd;  // rear-left   (CW)
        double m4_raw = throttle + roll_cmd + pitch_cmd;  // rear-right  (CCW)

        // 2. Find the highest and lowest commanded values
        double max_motor = std::max({m1_raw, m2_raw, m3_raw, m4_raw});
        double min_motor = std::min({m1_raw, m2_raw, m3_raw, m4_raw});

        // 3. Air Mode Scaling: Preserve the difference by shifting the baseline
        double shift = 0.0;
        if (max_motor > 1.0) {
            shift = 1.0 - max_motor; // Shift down so max is exactly 1.0
        } else if (min_motor < 0.0) {
            shift = 0.0 - min_motor; // Shift up so min is exactly 0.0
        }

        // Apply the shift to all motors equally
        double m1 = m1_raw + shift;
        double m2 = m2_raw + shift;
        double m3 = m3_raw + shift;
        double m4 = m4_raw + shift;

        // 4. Hard clamp as a final safety 
        m1 = std::max(0.0, std::min(1.0, m1));
        m2 = std::max(0.0, std::min(1.0, m2));
        m3 = std::max(0.0, std::min(1.0, m3));
        m4 = std::max(0.0, std::min(1.0, m4));

        esc.write(m1, m2, m3, m4);
    }

    void disarm() {
        esc.write(0.0, 0.0, 0.0, 0.0);
    }

private:
    IESC& esc;
};