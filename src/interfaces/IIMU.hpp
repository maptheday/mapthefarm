#pragma once

// ============================================================
//  IIMU  –  Interface (abstract contract) for any IMU sensor
//
//  This is the "slip of paper" that lists what every IMU
//  must be able to do, regardless of whether it is real
//  hardware or a software simulation.
//
//  Any class that inherits from IIMU MUST implement these
//  three methods, or the compiler will refuse to build.
// ============================================================

class IIMU {
public:
    virtual ~IIMU() = default;

    // One-time startup (wire up hardware, set sampling rates, etc.)
    virtual void   initialize()    = 0;

    // Read the current roll angle in degrees  (+/- 180)
    virtual double readGyroRoll()  = 0;

    // Read the current pitch angle in degrees (+/- 180)
    virtual double readGyroPitch() = 0;
};