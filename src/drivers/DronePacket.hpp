#pragma once

// ============================================================
//  DronePacket
//
//  The command packet that MissionCommander produces each
//  Core 0 cycle and writes into the IPC bridge.
//
//  Core 1 reads this to know what the drone should be doing
//  right now — armed or not, what attitude to hold, what
//  throttle to apply.
//
//  All angle targets in degrees.
//  throttle is 0.0 (stopped) to 1.0 (full).
// ============================================================
struct DronePacket {
    bool   armed       = false;
    double throttle    = 0.0;

    double targetRoll  = 0.0;   // degrees, 0 = level
    double targetPitch = 0.0;   // degrees, 0 = level
    double targetYaw   = 0.0;   // degrees, 0 = hold launch heading

    // If true, Core 0 wants Core 1 to hold the current yaw
    // rather than rotate toward targetYaw. Set this when the
    // mission hasn't issued an explicit yaw command — it tells
    // the yaw PID "stay wherever you are" instead of snapping
    // back to 0 degrees.
    bool holdYaw = true;
};