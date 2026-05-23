#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <vector>
#include <mutex>
#include <atomic>

#include "core/PID.hpp"
#include "interfaces/IIMU.hpp"
#include "interfaces/IBarometer.hpp"
#include "drivers/SimulatedIMU.hpp"
#include "drivers/SimulatedBarometer.hpp"
#include "drivers/MissionCommander.hpp"
#include "drivers/WindEvent.hpp"
#include "drivers/DronePacket.hpp"
#include "interfaces/IPlatformLauncher.hpp"

// ============================================================
//  FlightControllerProgram (Dual-Core Architecture)
//
//  CORE 0 (Avionics / Navigation): Runs at 20 Hz (50ms)
//  CORE 1 (Flight / Stabilization): Runs at 250 Hz (4ms)
//
//  Core launching is handled by an IPlatformLauncher so this
//  class has zero platform-specific code. Pass a ThreadLauncher
//  on Mac, an EspLauncher when flashing to the ESP32-S3.
// ============================================================

class FlightControllerProgram {
private:
    // ── Simulated hardware chips ──────────────────────────────
    SimulatedIMU        hardware_imu;
    SimulatedBarometer  hardware_baro;

    IIMU*       imu  = nullptr;
    IBarometer* baro = nullptr;

    MissionCommander mission;

    PIDController altitude_pid;
    PIDController roll_pid;
    PIDController pitch_pid;

    // ── 1. The Physical Universe (Simulation Data) ────────────
    std::mutex physics_mutex;
    double altitude       = 0.0;
    double vertical_vel   = 0.0;
    double roll_angle     = 0.0;
    double roll_velocity  = 0.0;
    double pitch_angle    = 0.0;
    double pitch_velocity = 0.0;

    // ── 2. The Inter-Core Bridge (IPC) ────────────────────────
    std::mutex ipc_mutex;
    struct {
        double target_roll   = 0.0;
        double target_pitch  = 0.0;
        double throttle_cmd  = 0.0;
        bool   armed         = false;
    } ipc_bridge;

    // ── System Control ────────────────────────────────────────
    std::atomic<bool> simulation_running{true};
    int cycle_core0 = 0;

    // ── Mission Timing & Wind Schedule ────────────────────────
    MissionCommander::Stage lastStage  = MissionCommander::Stage::WAIT_FOR_ARM;
    double                  stageTimer = 0.0;

    std::vector<WindEvent> wind_schedule = {
        { MissionCommander::Stage::TAKEOFF, 1.5,  -15.0,   0.0 },
        { MissionCommander::Stage::HOVER,   1.0,    0.0,  12.0 },
        { MissionCommander::Stage::HOVER,   3.5,   10.0,  -8.0 },
    };

    void applyWindEvents() {
        for (WindEvent& g : wind_schedule) {
            if (!g.fired && mission.getStage() == g.stage && stageTimer >= g.stageTimeSec) {
                std::cout << "\n>>> [WIND GUST]"
                          << "  stage=" << mission.stageName()
                          << "  t+" << g.stageTimeSec << "s"
                          << "  roll+" << g.rollDeg << "°"
                          << "  pitch+" << g.pitchDeg << "° <<<\n\n";

                std::lock_guard<std::mutex> lock(physics_mutex);
                roll_angle    += g.rollDeg;
                pitch_angle   += g.pitchDeg;
                roll_velocity += g.rollDeg  * 2.0;
                pitch_velocity+= g.pitchDeg * 2.0;

                g.fired = true;
            }
        }
    }

public:
    FlightControllerProgram() {
        imu  = &hardware_imu;
        baro = &hardware_baro;

        altitude_pid.Kp = 1.2;  altitude_pid.Ki = 0.05; altitude_pid.Kd = 0.8;
        altitude_pid.maxOutput = 0.4;

        roll_pid.Kp  = 8.0;  roll_pid.Ki  = 0.1;  roll_pid.Kd  = 3.0;
        pitch_pid.Kp = 8.0;  pitch_pid.Ki = 0.1;  pitch_pid.Kd = 3.0;
    }

    void setup() {
        std::cout << "========================================================\n";
        std::cout << "         DRONE FLIGHT CONTROLLER (DUAL-CORE)            \n";
        std::cout << "         Core 0: Avionics (20Hz) | Core 1: Flight (250Hz)\n";
        std::cout << "========================================================\n\n";

        imu->initialize();
        baro->initialize();

        std::cout << std::left
                  << std::setw(6)  << "Cycle"
                  << std::setw(11) << "Stage"
                  << std::setw(10) << "Alt(m)"
                  << std::setw(10) << "Roll°"
                  << std::setw(10) << "Pitch°"
                  << std::setw(10) << "Throttle"
                  << "Notes\n";
        std::cout << std::string(68, '-') << "\n";
    }

    // ============================================================
    //  CORE 0: Avionics & Navigation (Runs at 20 Hz)
    // ============================================================
    void runCore0_Avionics() {
        const double dt_core0 = 0.05; // 50 ms

        while (simulation_running) {
            cycle_core0++;

            // 1. Read sensors
            double measured_alt, measured_roll, measured_pitch;
            {
                std::lock_guard<std::mutex> lock(physics_mutex);

                hardware_baro.injectAltitude(altitude);
                hardware_imu.injectState(roll_angle, pitch_angle);

                measured_alt   = baro->readAltitudeMeters();
                measured_roll  = imu->readGyroRoll();
                measured_pitch = imu->readGyroPitch();
            }

            // 2. Mission Logic & Wind Timer
            MissionCommander::Stage currentStage = mission.getStage();
            if (currentStage != lastStage) {
                stageTimer = 0.0;
                lastStage  = currentStage;
            }
            stageTimer += dt_core0;
            applyWindEvents();

            DronePacket cmd = mission.update(measured_alt, measured_roll, measured_pitch, dt_core0);

            // 3. Altitude PID (Outer Loop)
            static double target_altitude = 0.0;

            if (mission.getStage() == MissionCommander::Stage::TAKEOFF ||
                mission.getStage() == MissionCommander::Stage::HOVER) {
                target_altitude = MissionCommander::TARGET_ALTITUDE_M;
            }
            else if (mission.getStage() != MissionCommander::Stage::WAIT_FOR_ARM &&
                       mission.getStage() != MissionCommander::Stage::COUNTDOWN) {
                target_altitude -= 0.5 * dt_core0;
                if (target_altitude < 0.0) target_altitude = 0.0;
            }

            double throttle_trim  = altitude_pid.calculate(target_altitude, measured_alt, dt_core0);
            double final_throttle = cmd.throttle + throttle_trim;

            if (final_throttle > 1.0) final_throttle = 1.0;
            if (final_throttle < 0.0) final_throttle = 0.0;

            // 4. Write to IPC bridge
            {
                std::lock_guard<std::mutex> lock(ipc_mutex);
                ipc_bridge.target_roll  = cmd.targetRoll;
                ipc_bridge.target_pitch = cmd.targetPitch;
                ipc_bridge.throttle_cmd = final_throttle;
                ipc_bridge.armed        = cmd.armed;
            }

            // 5. Telemetry
            bool stable = (measured_roll > -1.0 && measured_roll < 1.0) &&
                          (measured_pitch > -1.0 && measured_pitch < 1.0);

            std::cout << std::right << std::fixed << std::setprecision(2)
                      << std::setw(4)  << cycle_core0 << "  "
                      << std::left  << std::setw(11) << mission.stageName()
                      << std::right << std::setw(7)  << measured_alt   << "m  "
                      << std::setw(7)  << measured_roll  << "°  "
                      << std::setw(7)  << measured_pitch << "°  "
                      << std::setw(7)  << final_throttle << "  "
                      << (stable ? "STABLE" : "") << "\n";

            if (mission.isComplete() || cycle_core0 > 1000) {
                simulation_running = false;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    // ============================================================
    //  CORE 1: Flight Stabilization (Runs at 250 Hz)
    // ============================================================
    void runCore1_Flight() {
        const double dt_core1 = 0.004; // 4 ms

        while (simulation_running) {

            // 1. Read from IPC bridge
            double target_roll, target_pitch, throttle_cmd;
            bool is_armed;
            {
                std::lock_guard<std::mutex> lock(ipc_mutex);
                target_roll  = ipc_bridge.target_roll;
                target_pitch = ipc_bridge.target_pitch;
                throttle_cmd = ipc_bridge.throttle_cmd;
                is_armed     = ipc_bridge.armed;
            }

            // 2. Read IMU
            double measured_roll, measured_pitch;
            {
                std::lock_guard<std::mutex> lock(physics_mutex);

                hardware_imu.injectState(roll_angle, pitch_angle);

                measured_roll  = imu->readGyroRoll();
                measured_pitch = imu->readGyroPitch();
            }

            // 3. Attitude PIDs (Inner Loop)
            double roll_cmd  = roll_pid.calculate(target_roll, measured_roll, dt_core1);
            double pitch_cmd = pitch_pid.calculate(target_pitch, measured_pitch, dt_core1);

            // 4. Update Physical World
            {
                std::lock_guard<std::mutex> lock(physics_mutex);
                if (is_armed) {
                    double vertical_accel = (throttle_cmd - 0.5) * 20.0;
                    vertical_vel = (vertical_vel + vertical_accel * dt_core1) * 0.92;
                } else {
                    vertical_vel = (vertical_vel - 9.8 * dt_core1) * 0.95;
                }

                altitude += vertical_vel * dt_core1;
                if (altitude < 0.0) { altitude = 0.0; vertical_vel = 0.0; }

                const double inertia = 0.1;
                const double drag    = 0.88;
                roll_velocity  = (roll_velocity  + (roll_cmd  / inertia) * dt_core1) * drag;
                pitch_velocity = (pitch_velocity + (pitch_cmd / inertia) * dt_core1) * drag;
                roll_angle  += roll_velocity  * dt_core1;
                pitch_angle += pitch_velocity * dt_core1;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(4));
        }
    }

    // ============================================================
    //  runDualCore — platform-agnostic launch
    //
    //  Pass a ThreadLauncher for Mac simulation.
    //  Pass an EspLauncher when running on the ESP32-S3.
    //  This method has no #ifdef and no platform knowledge.
    // ============================================================
    void runDualCore(IPlatformLauncher& launcher) {
        launcher.launchCore0([this]() { runCore0_Avionics(); });
        launcher.launchCore1([this]() { runCore1_Flight(); });
        launcher.waitForCompletion();
    }
};