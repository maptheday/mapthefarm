#pragma once

// ============================================================
//  IIMU  —  Inertial Measurement Unit interface
//
//  Abstracts reading orientation angles so FlightControllerProgram
//  has no idea whether it's talking to a simulated IMU or a
//  real chip (e.g. MPU6050, ICM-42688).
//
//  Axes:
//    Roll  — rotation around the front-back axis  (left/right lean)
//    Pitch — rotation around the side-side axis   (nose up/down)
//    Yaw   — rotation around the vertical axis    (left/right turn)
//
//  All angles in degrees.
//  Yaw is in the range -180 to +180 (0 = starting heading).
//
//  On simulation:
//    All three are fed from the physics simulation state.
//
//  On real hardware:
//    Roll and pitch come from accelerometer + gyro fusion.
//    Yaw comes from gyro integration or a magnetometer.
//    A magnetometer gives absolute heading (true north).
//    Gyro integration gives relative heading (drift over time).
// ============================================================
class IIMU {
public:
    virtual void initialize()      = 0;
    virtual double readGyroRoll()  = 0;   // degrees, + = right lean
    virtual double readGyroPitch() = 0;   // degrees, + = nose up
    virtual double readYawDeg()    = 0;   // degrees, -180 to +180
    virtual ~IIMU() = default;
};