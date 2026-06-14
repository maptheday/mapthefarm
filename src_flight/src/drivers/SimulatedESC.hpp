#pragma once
#include "../interfaces/IESC.hpp"
#include <iostream>
#include <iomanip>

// ============================================================
//  SimulatedESC  —  Mac / Linux simulation
//
//  Stores the last written motor values so the physics
//  simulation in Core 1 can read them back. No real PWM,
//  no hardware — just numbers in memory.
//
//  On real hardware this is replaced by an ESC driver that
//  writes PWM signals to the four ESC signal wires.
// ============================================================
class SimulatedESC : public IESC {
public:
    // Last commanded speed for each motor (0.0 – 1.0)
    double m1 = 0.0;
    double m2 = 0.0;
    double m3 = 0.0;
    double m4 = 0.0;

    void initialize() override {
        std::cout << "[ESC] Simulated ESC initialised (4 motors)\n";
        m1 = m2 = m3 = m4 = 0.0;
    }

    void write(double _m1, double _m2, double _m3, double _m4) override {
        m1 = _m1;
        m2 = _m2;
        m3 = _m3;
        m4 = _m4;
    }

    double getMotor(int index) const override {
        switch (index) {
            case 1: return m1;
            case 2: return m2;
            case 3: return m3;
            case 4: return m4;
            default: return 0.0;
        }
    }

    void disarm() override {
        m1 = m2 = m3 = m4 = 0.0;
    }
};