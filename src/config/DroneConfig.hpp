#pragma once

// ============================================================
//  DroneConfig
//
//  Physical properties of the drone. These values drive the
//  physics simulation in Core 1 so the sim reflects your
//  actual hardware instead of magic numbers.
//
//  Pass an instance into FlightControllerProgram so you can
//  swap drone configs without touching flight controller code.
//
//  Example — a typical 5-inch freestyle quad:
//    DroneConfig config {
//        .mass_kg          = 0.650,   // 650g all-up weight
//        .arm_length_m     = 0.120,   // 120mm center to motor
//        .motor_thrust_max = 9.0,     // ~900g thrust per motor
//    };
//
//  Example — a heavy 7-inch mapping drone:
//    DroneConfig config {
//        .mass_kg          = 1.400,
//        .arm_length_m     = 0.180,
//        .motor_thrust_max = 14.0,
//    };
// ============================================================

struct DroneConfig {
    // Total flying weight in kg — include frame, motors,
    // battery, camera, and any payload.
    double mass_kg;

    // Distance in metres from the centre of the frame to the
    // centre of each motor. Longer arms = more stable but
    // more sluggish to rotate (higher moment of inertia).
    double arm_length_m;

    // Maximum thrust in Newtons that one motor can produce
    // at 100% throttle. Multiply by 4 for total thrust.
    // A rough conversion: 1 kg of thrust ≈ 9.81 N.
    double motor_thrust_max;

    // ── Derived values (calculated once on construction) ─────

    // The throttle fraction (0.0–1.0) at which thrust exactly
    // cancels gravity. Below this the drone descends, above it
    // the drone climbs. On a real drone this is what you tune
    // with "hover throttle" in Betaflight.
    double hover_throttle;

    // Moment of inertia (kg·m²). Treats the four motors as
    // point masses at arm_length_m from the centre.
    // I = mass * r^2  (simplified single-axis model)
    double inertia;

    // Total thrust available across all four motors (N).
    double total_thrust_max;

    DroneConfig(double mass, double arm, double thrust_per_motor)
        : mass_kg(mass)
        , arm_length_m(arm)
        , motor_thrust_max(thrust_per_motor)
    {
        total_thrust_max = 4.0 * motor_thrust_max;
        hover_throttle   = (mass_kg * 9.81) / total_thrust_max;
        inertia          = mass_kg * (arm_length_m * arm_length_m);
    }
};