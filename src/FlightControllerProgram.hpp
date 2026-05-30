#pragma once
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <vector>
#include <atomic>
#include <cmath>

#include "core/PID.hpp"
#include "core/MotorMixer.hpp"
#include "interfaces/IIMU.hpp"
#include "interfaces/IBarometer.hpp"
#include "interfaces/IESC.hpp"
#include "drivers/SimulatedIMU.hpp"
#include "drivers/SimulatedBarometer.hpp"
#include "drivers/SimulatedESC.hpp"
#include "drivers/MissionCommander.hpp"
#include "drivers/WindEvent.hpp"
#include "drivers/DronePacket.hpp"
#include "platform/IPlatformLauncher.hpp"
#include "platform/IPlatformMutex.hpp"
#include "config/DroneConfig.hpp"

#ifdef ON_REAL_HARDWARE
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

// ============================================================
//  FlightControllerProgram (Dual-Core Architecture)
//
//  CORE 0 (Avionics / Navigation): Runs at 20 Hz (50ms)
//  CORE 1 (Flight / Stabilization): Runs at 250 Hz (4ms)
//
//  Constructor variants:
//
//    Mac simulation (SimulatedESC used internally):
//      FlightControllerProgram program(config, physics_mx, ipc_mx);
//
//    ESP32-S3 hardware (real ESC + sensors injected):
//      FlightControllerProgram program(config, physics_mx, ipc_mx, esc, imu, baro);
// ============================================================

class FlightControllerProgram {
private:
    // ── Hardware drivers ──────────────────────────────────────
    SimulatedIMU       hardware_imu;
    SimulatedBarometer hardware_baro;
    SimulatedESC       simulated_esc;

    IIMU*       imu  = nullptr;
    IBarometer* baro = nullptr;
    IESC*       esc  = nullptr;

    MissionCommander mission;

    PIDController altitude_pid;
    PIDController roll_pid;
    PIDController pitch_pid;
    PIDController yaw_pid;       // NEW: controls rotation around vertical axis
    MotorMixer    mixer;

    // ── Physical properties of this drone ────────────────────
    const DroneConfig& config;

    // ── Platform mutexes ─────────────────────────────────────
    IPlatformMutex& physics_mutex;
    IPlatformMutex& ipc_mutex;

    // ── 1. The Physical Universe (Simulation State) ───────────
    double altitude       = 0.0;
    double vertical_vel   = 0.0;
    double roll_angle     = 0.0;
    double roll_velocity  = 0.0;
    double pitch_angle    = 0.0;
    double pitch_velocity = 0.0;
    double yaw_angle      = 0.0;  // NEW: heading in degrees (0 = north, 90 = east)
    double yaw_velocity   = 0.0;  // NEW: rotation rate in deg/s

    // ── 2. The Inter-Core Bridge (IPC) ────────────────────────
    struct {
        double target_roll   = 0.0;
        double target_pitch  = 0.0;
        double target_yaw    = 0.0;  // NEW: desired heading in degrees
        double throttle_cmd  = 0.0;
        bool   armed         = false;

        double motor_m1 = 0.0;
        double motor_m2 = 0.0;
        double motor_m3 = 0.0;
        double motor_m4 = 0.0;
    } ipc_bridge;

    // ── System Control ────────────────────────────────────────
    std::atomic<bool> simulation_running{true};
    int cycle_core0 = 0;

    // ── Mission Timing & Wind Schedule ────────────────────────
    MissionCommander::Stage lastStage  = MissionCommander::Stage::WAIT_FOR_ARM;
    double                  stageTimer = 0.0;

    std::vector<WindEvent> wind_schedule = {
        WindEvent(MissionCommander::Stage::TAKEOFF, 1.5, -15.0,   0.0),
        WindEvent(MissionCommander::Stage::HOVER,   1.0,   0.0,  12.0),
        WindEvent(MissionCommander::Stage::HOVER,   3.5,  10.0,  -8.0),
    };

    // ── Platform sleep helper ─────────────────────────────────
    void sleepMs(int ms) {
#ifdef ON_REAL_HARDWARE
        vTaskDelay(pdMS_TO_TICKS(ms));
#else
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
#endif
    }

    void applyWindEvents() {
        for (WindEvent& g : wind_schedule) {
            if (!g.fired && mission.getStage() == g.stage && stageTimer >= g.stageTimeSec) {
                std::cout << "\n>>> [WIND GUST]"
                          << "  stage=" << mission.stageName()
                          << "  t+"     << g.stageTimeSec << "s"
                          << "  roll+"  << g.rollDeg      << "deg"
                          << "  pitch+" << g.pitchDeg     << "deg <<<\n\n";

                PlatformLockGuard lock(physics_mutex);
                roll_angle     += g.rollDeg;
                pitch_angle    += g.pitchDeg;
                roll_velocity  += g.rollDeg  * 2.0;
                pitch_velocity += g.pitchDeg * 2.0;

                g.fired = true;
            }
        }
    }

    // ── Shared constructor body ───────────────────────────────
    void init() {
        if (!imu)  imu  = &hardware_imu;
        if (!baro) baro = &hardware_baro;
        if (!esc)  esc  = &simulated_esc;

        altitude_pid.Kp = 1.2;   altitude_pid.Ki = 0.05;  altitude_pid.Kd = 0.8;
        altitude_pid.maxOutput = 0.4;

        roll_pid.Kp  = 0.016;  roll_pid.Ki  = 0.001;  roll_pid.Kd  = 0.008;
        roll_pid.maxOutput = 0.8;

        pitch_pid.Kp = 0.016;  pitch_pid.Ki = 0.001;  pitch_pid.Kd = 0.008;
        pitch_pid.maxOutput = 0.8;

        // Yaw is slower to respond than roll/pitch — lower Kp, no Ki to
        // avoid windup during long holds, moderate Kd for smooth stops.
        yaw_pid.Kp = 0.010;  yaw_pid.Ki = 0.0;  yaw_pid.Kd = 0.005;
        yaw_pid.maxOutput = 0.3;  // yaw authority capped lower than roll/pitch
    }

public:
    // ── Constructor (simulation) ──────────────────────────────
    FlightControllerProgram(const DroneConfig& cfg,
                            IPlatformMutex& physics_mx,
                            IPlatformMutex& ipc_mx)
        : config(cfg)
        , physics_mutex(physics_mx)
        , ipc_mutex(ipc_mx)
        , mixer(simulated_esc)
    {
        init();
    }

    // ── Constructor (hardware) ────────────────────────────────
    FlightControllerProgram(const DroneConfig& cfg,
                            IPlatformMutex& physics_mx,
                            IPlatformMutex& ipc_mx,
                            IESC& real_esc,
                            IIMU& real_imu,
                            IBarometer& real_baro)
        : config(cfg)
        , physics_mutex(physics_mx)
        , ipc_mutex(ipc_mx)
        , mixer(real_esc)
    {
        esc  = &real_esc;
        imu  = &real_imu;
        baro = &real_baro;
        init();
    }

    void setup() {
        std::cout << "========================================================\n";
        std::cout << "         DRONE FLIGHT CONTROLLER (DUAL-CORE)            \n";
        std::cout << "         Core 0: Avionics (20Hz) | Core 1: Flight (250Hz)\n";
        std::cout << "         Mass: "   << config.mass_kg << "kg"
                  << "  Arm: "           << config.arm_length_m * 1000.0 << "mm"
                  << "  Hover: "         << (int)(config.hover_throttle * 100.0) << "% throttle\n";
        std::cout << "========================================================\n\n";

        imu->initialize();
        baro->initialize();
        esc->initialize();

        std::cout << std::left
                  << std::setw(6)  << "Cycle"
                  << std::setw(11) << "Stage"
                  << std::setw(10) << "Alt(m)"
                  << std::setw(10) << "Roll"
                  << std::setw(10) << "Pitch"
                  << std::setw(10) << "Yaw"
                  << std::setw(10) << "Throttle"
                  << std::setw(24) << "Motors (M1 M2 M3 M4)"
                  << "Notes\n";
        std::cout << std::string(100, '-') << "\n";
    }

    // ============================================================
    //  CORE 0: Avionics & Navigation (20 Hz)
    // ============================================================
    void runCore0_Avionics() {
        const double dt = 0.05;

        while (simulation_running) {
            cycle_core0++;

            // 1. Read sensors
            double measured_alt, measured_roll, measured_pitch, measured_yaw;
            {
                PlatformLockGuard lock(physics_mutex);
                hardware_baro.injectAltitude(altitude);
                hardware_imu.injectState(roll_angle, pitch_angle, yaw_angle);
                measured_alt   = baro->readAltitudeMeters();
                measured_roll  = imu->readGyroRoll();
                measured_pitch = imu->readGyroPitch();
                measured_yaw   = imu->readYawDeg();  // sim: from injected state, hardware: from real IMU
            }

            // 2. Mission logic & wind events
            MissionCommander::Stage currentStage = mission.getStage();
            if (currentStage != lastStage) {
                stageTimer = 0.0;
                lastStage  = currentStage;
                if (currentStage == MissionCommander::Stage::LAND) {
                    target_altitude_hold = 0.0;
                }
            }
            stageTimer += dt;
            applyWindEvents();

            DronePacket cmd = mission.update(measured_alt, measured_roll, measured_pitch, dt);

            // 3. Altitude PID (outer loop)
            if (mission.getStage() == MissionCommander::Stage::TAKEOFF ||
                mission.getStage() == MissionCommander::Stage::HOVER) {
                target_altitude_hold = MissionCommander::TARGET_ALTITUDE_M;
            }
            else if (mission.getStage() != MissionCommander::Stage::WAIT_FOR_ARM &&
                     mission.getStage() != MissionCommander::Stage::COUNTDOWN &&
                     mission.getStage() != MissionCommander::Stage::LAND) {
                target_altitude_hold -= 0.5 * dt;
                if (target_altitude_hold < 0.0) target_altitude_hold = 0.0;
            }

            double throttle_trim  = altitude_pid.calculate(target_altitude_hold, measured_alt, dt);
            double final_throttle = cmd.throttle + throttle_trim;

            if (final_throttle > 1.0) final_throttle = 1.0;
            if (cmd.armed && final_throttle < 0.05) final_throttle = 0.05;
            else if (!cmd.armed && final_throttle < 0.0) final_throttle = 0.0;

            // 4. Write to IPC bridge
            //    target_yaw holds the last commanded heading. When the mission
            //    doesn't issue a yaw command we hold whatever heading we're at,
            //    so we pass measured_yaw as the target to keep the drone pointed
            //    in the same direction rather than drifting.
            {
                PlatformLockGuard lock(ipc_mutex);
                ipc_bridge.target_roll  = cmd.targetRoll;
                ipc_bridge.target_pitch = cmd.targetPitch;
                // If the mission hasn't issued an explicit yaw command, hold
                // the current heading rather than snapping back to 0 degrees.
                ipc_bridge.target_yaw   = cmd.holdYaw ? measured_yaw : cmd.targetYaw;
                ipc_bridge.throttle_cmd = final_throttle;
                ipc_bridge.armed        = cmd.armed;
            }

            // 5. Telemetry
            double m1, m2, m3, m4;
            {
                PlatformLockGuard lock(ipc_mutex);
                m1 = ipc_bridge.motor_m1;
                m2 = ipc_bridge.motor_m2;
                m3 = ipc_bridge.motor_m3;
                m4 = ipc_bridge.motor_m4;
            }

            bool stable = (measured_roll  > -1.0 && measured_roll  < 1.0) &&
                          (measured_pitch > -1.0 && measured_pitch < 1.0);

            std::cout << std::right << std::fixed << std::setprecision(2)
                      << std::setw(4)  << cycle_core0      << "  "
                      << std::left  << std::setw(11) << mission.stageName()
                      << std::right << std::setw(7)  << measured_alt   << "m  "
                      << std::setw(7)  << measured_roll    << "deg  "
                      << std::setw(7)  << measured_pitch   << "deg  "
                      << std::setw(7)  << measured_yaw     << "deg  "
                      << std::setw(7)  << final_throttle   << "  "
                      << std::setw(5)  << m1 << " "
                      << std::setw(5)  << m2 << " "
                      << std::setw(5)  << m3 << " "
                      << std::setw(5)  << m4 << "  "
                      << (stable ? "STABLE" : "") << "\n";

            if (mission.isComplete() || cycle_core0 > 1000) {
                simulation_running = false;
            }

            sleepMs(50);
        }
    }

    // ============================================================
    //  CORE 1: Flight Stabilization (250 Hz)
    // ============================================================
    void runCore1_Flight() {
        const double dt = 0.004;

        while (simulation_running) {

            // 1. Read from IPC bridge
            double target_roll, target_pitch, target_yaw, throttle_cmd;
            bool is_armed;
            {
                PlatformLockGuard lock(ipc_mutex);
                target_roll  = ipc_bridge.target_roll;
                target_pitch = ipc_bridge.target_pitch;
                target_yaw   = ipc_bridge.target_yaw;
                throttle_cmd = ipc_bridge.throttle_cmd;
                is_armed     = ipc_bridge.armed;
            }

            // 2. Read IMU
            double measured_roll, measured_pitch, measured_yaw;
            {
                PlatformLockGuard lock(physics_mutex);
                hardware_imu.injectState(roll_angle, pitch_angle, yaw_angle);
                measured_roll  = imu->readGyroRoll();
                measured_pitch = imu->readGyroPitch();
                measured_yaw   = imu->readYawDeg();  // sim: injected, hardware: real IMU chip
            }

            // 3. Attitude PIDs — now includes yaw
            double roll_cmd  = roll_pid.calculate(target_roll,  measured_roll,  dt);
            double pitch_cmd = pitch_pid.calculate(target_pitch, measured_pitch, dt);
            double yaw_cmd   = yaw_pid.calculate(target_yaw,   measured_yaw,   dt);

            // 4. Motor mixer — now passes yaw_cmd as the fourth argument
            if (is_armed) {
                mixer.mix(throttle_cmd, roll_cmd, pitch_cmd, yaw_cmd);
            } else {
                mixer.disarm();
            }

            // Write motor snapshot back to IPC bridge for telemetry
            {
                PlatformLockGuard lock(ipc_mutex);
                ipc_bridge.motor_m1 = esc->getMotor(1);
                ipc_bridge.motor_m2 = esc->getMotor(2);
                ipc_bridge.motor_m3 = esc->getMotor(3);
                ipc_bridge.motor_m4 = esc->getMotor(4);
            }

            // 5. Update physics (simulation only — not compiled on hardware)
#ifndef ON_REAL_HARDWARE
            {
                PlatformLockGuard lock(physics_mutex);

                if (is_armed) {
                    double avg_throttle = (esc->getMotor(1) + esc->getMotor(2) +
                                          esc->getMotor(3) + esc->getMotor(4)) / 4.0;

                    double roll_rad    = roll_angle  * M_PI / 180.0;
                    double pitch_rad   = pitch_angle * M_PI / 180.0;
                    double tilt_cos    = std::cos(roll_rad) * std::cos(pitch_rad);
                    double raw_thrust  = avg_throttle * config.total_thrust_max;
                    double vert_thrust = raw_thrust * tilt_cos;
                    double gravity     = config.mass_kg * 9.81;
                    double vert_accel  = (vert_thrust - gravity) / config.mass_kg;
                    vertical_vel = (vertical_vel + vert_accel * dt) * 0.985;
                } else {
                    vertical_vel = (vertical_vel - 9.81 * dt) * 0.99;
                }

                altitude += vertical_vel * dt;
                if (altitude < 0.0) { altitude = 0.0; vertical_vel = 0.0; }

                // Roll and pitch torque from differential motor thrust
                double roll_torque  = (esc->getMotor(2) + esc->getMotor(4))
                                    - (esc->getMotor(1) + esc->getMotor(3));
                double pitch_torque = (esc->getMotor(3) + esc->getMotor(4))
                                    - (esc->getMotor(1) + esc->getMotor(2));

                // Yaw torque from diagonal motor pairs.
                // CCW motors (M1, M4) produce CW frame reaction (+yaw).
                // CW  motors (M2, M3) produce CCW frame reaction (-yaw).
                double yaw_torque = (esc->getMotor(1) + esc->getMotor(4))
                                  - (esc->getMotor(2) + esc->getMotor(3));

                const double drag = 0.96;
                roll_velocity  = (roll_velocity  + (roll_torque  / config.inertia) * dt) * drag;
                pitch_velocity = (pitch_velocity + (pitch_torque / config.inertia) * dt) * drag;

                // Yaw inertia is higher than roll/pitch because it involves
                // the whole frame rotating, not just tilting.
                // We approximate it as 2x the roll/pitch inertia.
                const double yaw_inertia = config.inertia * 2.0;
                yaw_velocity  = (yaw_velocity + (yaw_torque / yaw_inertia) * dt) * drag;

                roll_angle  += roll_velocity  * dt;
                pitch_angle += pitch_velocity * dt;
                yaw_angle   += yaw_velocity   * dt;

                // Keep yaw in -180 to +180 range
                if (yaw_angle >  180.0) yaw_angle -= 360.0;
                if (yaw_angle < -180.0) yaw_angle += 360.0;
            }
#endif

            sleepMs(4);
        }
    }

    // ============================================================
    //  runDualCore — platform-agnostic launch
    // ============================================================
    void runDualCore(IPlatformLauncher& launcher) {
        launcher.launchCore0([this]() { runCore0_Avionics(); });
        launcher.launchCore1([this]() { runCore1_Flight(); });
        launcher.waitForCompletion();
    }

private:
    double target_altitude_hold = 0.0;  // moved to member so LAND stage can reset it
};