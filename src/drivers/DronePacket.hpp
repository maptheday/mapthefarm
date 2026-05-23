#pragma once

// ============================================================
//  DronePacket
//
//  This is the exact data structure the Synapse transmitter
//  sends to the drone over nRF24 radio every cycle.
//
//  In simulation, MissionCommander fills this struct instead
//  of real radio hardware.  On real hardware, your radio task
//  would receive bytes over nRF24 and deserialise them into
//  this same struct.  The flight controller never knows the
//  difference.
//
//  All angles in degrees.  Throttle is 0.0 – 1.0 (0% – 100%).
// ============================================================

struct DronePacket {
    double throttle    = 0.0;   // 0.0 = motors off, 1.0 = full power
    double targetRoll  = 0.0;   // degrees  (inner loop setpoint)
    double targetPitch = 0.0;   // degrees  (inner loop setpoint)
    double targetYaw   = 0.0;   // degrees  (inner loop setpoint, not used yet)
    bool   armed       = false; // safety: motors only spin when true
};