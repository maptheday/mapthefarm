#pragma once

// ============================================================
//  IBarometer  –  Interface for any barometric pressure sensor
//
//  On real hardware this would be a BMP280 chip talking over I2C.
//  In simulation it's a fake that we push values into.
//
//  The flight controller only ever calls readAltitudeMeters().
//  It never knows (or cares) if it's talking to real silicon or
//  a software pretender.
// ============================================================

class IBarometer {
public:
    virtual ~IBarometer() = default;

    // One-time startup
    virtual void   initialize()          = 0;

    // Returns current altitude above ground in meters
    virtual double readAltitudeMeters()  = 0;
};