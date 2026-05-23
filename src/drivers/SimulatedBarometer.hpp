#pragma once
#include <iostream>
#include "../interfaces/IBarometer.hpp"

// ============================================================
//  SimulatedBarometer
//
//  Pretends to be a BMP280 chip.
//
//  The physics simulation calls injectAltitude() every cycle
//  with the drone's true height.  readAltitudeMeters() just
//  returns it, exactly like a real chip reading air pressure
//  and converting it to meters.
//
//  Real BMP280 swap:
//    Replace this class with one that calls bmp.readAltitude()
//    over I2C.  Everything else stays the same.
// ============================================================

class SimulatedBarometer : public IBarometer {
public:

    void initialize() override {
        std::cout << "[BARO] SimulatedBarometer (BMP280) initialised.\n";
        std::cout << "[BARO] Reporting altitude in meters above ground.\n";
    }

    // Called by the physics simulation each cycle
    void injectAltitude(double altitudeMeters) {
        _altitude = altitudeMeters;
    }

    // Called by the flight controller through the IBarometer interface
    double readAltitudeMeters() override {
        return _altitude;
    }

private:
    double _altitude = 0.0;
};