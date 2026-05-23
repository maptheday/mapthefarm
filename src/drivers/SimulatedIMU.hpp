#pragma once
#include <iostream>
#include "../interfaces/IIMU.hpp"

// ============================================================
//  SimulatedIMU
//
//  Pretends to be a real MPU6050 chip but lives entirely in
//  software.  The flight controller talks to it through the
//  IIMU interface and doesn't know (or care) it's fake.
//
//  How it works:
//    1.  The physics simulation calls injectState() every cycle
//        to push the "true" angles the drone is currently at.
//    2.  readGyroRoll() / readGyroPitch() just return those
//        values – exactly like a real chip reading its registers.
//
//  On real hardware you would replace this class with one that
//  talks to an actual MPU6050 over I2C.  The rest of the code
//  doesn't change at all.
// ============================================================

class SimulatedIMU : public IIMU {
public:

    void initialize() override {
        std::cout << "[IMU] SimulatedIMU initialised.\n";
        std::cout << "[IMU] Reporting roll and pitch axes.\n";
    }

    // Called by the physics simulation – NOT by the controller
    void injectState(double rollDeg, double pitchDeg) {
        _roll  = rollDeg;
        _pitch = pitchDeg;
    }

    // Called by the flight controller through the IIMU interface
    double readGyroRoll()  override { return _roll;  }
    double readGyroPitch() override { return _pitch; }

private:
    double _roll  = 0.0;
    double _pitch = 0.0;
};