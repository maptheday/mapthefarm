#pragma once
#include "../interfaces/IIMU.hpp"
#include <iostream>
#include <cstdlib>

// ============================================================
//  SimulatedIMU  —  Mac / Linux simulation
//
//  Stores injected roll, pitch, and yaw angles from the
//  physics simulation. Core 1 calls injectState() each cycle
//  to push the current sim state in, then reads back via the
//  IIMU interface methods.
//
//  A small amount of gaussian noise is added to simulate
//  real sensor noise — real IMUs are never perfectly clean.
// ============================================================
class SimulatedIMU : public IIMU {
public:
    void initialize() override {
        std::cout << "[IMU] SimulatedIMU initialised.\n";
        std::cout << "[IMU] Reporting roll, pitch, and yaw axes.\n";
    }

    // Called by Core 1 physics block each cycle to keep
    // the simulated sensor state in sync with the fake world.
    void injectState(double roll_deg, double pitch_deg, double yaw_deg = 0.0) {
        injected_roll  = roll_deg;
        injected_pitch = pitch_deg;
        injected_yaw   = yaw_deg;
    }

    double readGyroRoll()  override { return injected_roll  + noise(); }
    double readGyroPitch() override { return injected_pitch + noise(); }
    double readYawDeg()    override { return injected_yaw   + noise(); }

private:
    double injected_roll  = 0.0;
    double injected_pitch = 0.0;
    double injected_yaw   = 0.0;

    // Small random noise to simulate real sensor imprecision.
    // Range: roughly ±0.05 degrees.
    static double noise() {
        return ((double)rand() / RAND_MAX - 0.5) * 0.1;
    }
};