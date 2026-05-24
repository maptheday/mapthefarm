#pragma once
#include <string>
#include <iostream>
#include "DronePacket.hpp"

// ============================================================
//  MissionCommander
//
//  Replaces the human pilot + Synapse transmitter.
//
//  ── How arming works ────────────────────────────────────────
//
//  In SIMULATION:
//    The arm pin is always treated as "HIGH" (pressed).
//    The countdown runs automatically so you can watch it.
//
//  On REAL ESP32 HARDWARE:
//    1. Wire a physical toggle switch between GPIO pin and 3.3V
//       (with a 10k pull-down resistor to GND).
//    2. Set ARM_PIN to that GPIO number in your hardware main.cpp
//    3. Replace the isArmPinHigh() stub with:
//
//         bool isArmPinHigh() {
//             return digitalRead(ARM_PIN) == HIGH;
//         }
//
//    The drone will sit in PREFLIGHT forever until you flip
//    the switch.  Once flipped, the countdown begins.
//    If you flip it back during countdown, it resets.
//    Only after full countdown does it arm and take off.
//
//  ── Mission stages ──────────────────────────────────────────
//
//  WAIT_FOR_ARM    Sits idle until arm pin goes HIGH
//  COUNTDOWN       10-second audible countdown, backs off if pin drops
//  TAKEOFF         Climb to TARGET_ALTITUDE_M
//  HOVER           Hold altitude for HOVER_DURATION_SEC
//  LAND            Descend back to ground
//  SHUTDOWN        Motors off, mission complete
//
//  ── Safety hard limits ───────────────────────────────────────
//
//  MAX_ALTITUDE_M  Absolute ceiling — cuts throttle if breached
//  MAX_TILT_DEG    Abort if drone tilts past this angle
// ============================================================

class MissionCommander {
public:
    // ── Mission parameters ───────────────────────────────────
    static constexpr double TARGET_ALTITUDE_M  = 1.5;
    static constexpr double HOVER_DURATION_SEC = 5.0;

    // ── Countdown duration ───────────────────────────────────
    // How many seconds to count down after arm switch is flipped.
    // You want enough time to step well back from the drone.
    static constexpr double LAUNCH_COUNTDOWN_SEC = 10.0;

    // ── Safety limits ────────────────────────────────────────
    static constexpr double MAX_ALTITUDE_M = 3.0;
    static constexpr double MAX_TILT_DEG   = 45.0;

    // ── Stages ───────────────────────────────────────────────
    enum class Stage {
        WAIT_FOR_ARM,
        COUNTDOWN,
        TAKEOFF,
        HOVER,
        LAND,
        SHUTDOWN
    };

    // ── Main update — called every cycle ─────────────────────
    DronePacket update(double currentAltitude,
                       double currentRoll,
                       double currentPitch,
                       double dt)
    {
        DronePacket cmd;

        // ── Safety checks ─────────────────────────────────────
        if (_stage != Stage::WAIT_FOR_ARM &&
            _stage != Stage::COUNTDOWN    &&
            _stage != Stage::SHUTDOWN)
        {
            if (currentAltitude > MAX_ALTITUDE_M + 0.05) {
                std::cout << "\n!!! [SAFETY] Altitude ceiling breached ("
                          << currentAltitude << "m > " << MAX_ALTITUDE_M
                          << "m) — EMERGENCY CUT !!!\n";
                _stage = Stage::SHUTDOWN;
            }

            double absTilt = absVal(currentRoll) + absVal(currentPitch);
            if (absTilt > MAX_TILT_DEG) {
                std::cout << "\n!!! [SAFETY] Excessive tilt (" << absTilt
                          << "°) — EMERGENCY ABORT !!!\n";
                _stage = Stage::SHUTDOWN;
            }
        }

        // ── Stage machine ─────────────────────────────────────
        switch (_stage) {
            case Stage::WAIT_FOR_ARM: runWaitForArm(cmd, dt);                        break;
            case Stage::COUNTDOWN:    runCountdown(cmd, dt);                        break;
            case Stage::TAKEOFF:      runTakeoff(cmd, currentAltitude, currentRoll, currentPitch); break;
            case Stage::HOVER:        runHover(cmd, currentAltitude, dt);           break;
            case Stage::LAND:         runLand(cmd, currentAltitude, currentRoll, currentPitch);  break;
            case Stage::SHUTDOWN:     cmd.armed = false; cmd.throttle = 0.0; break;
        }

        return cmd;
    }

    Stage       getStage()   const { return _stage; }
    bool        isComplete() const { return _stage == Stage::SHUTDOWN; }

    std::string stageName()  const {
        switch (_stage) {
            case Stage::WAIT_FOR_ARM: return "WAIT_ARM ";
            case Stage::COUNTDOWN:    return "COUNTDOWN";
            case Stage::TAKEOFF:      return "TAKEOFF  ";
            case Stage::HOVER:        return "HOVER    ";
            case Stage::LAND:         return "LAND     ";
            case Stage::SHUTDOWN:     return "SHUTDOWN ";
        }
        return "UNKNOWN  ";
    }

private:
    Stage  _stage           = Stage::WAIT_FOR_ARM;
    double _countdownTimer  = 0.0;
    double _hoverTimer      = 0.0;
    int    _lastBeepSecond  = -1;   // tracks which second we last printed
    double _waitPrintTimer  = 0.0;  // throttles "waiting..." messages
    double _landStartRoll   = 0.0;  // freeze attitude during landing
    double _landStartPitch  = 0.0;  // to prevent PID-driven lift

    // ── isArmPinHigh() ────────────────────────────────────────
    //
    //  SIMULATION: always returns true so mission runs automatically.
    //
    //  REAL HARDWARE — replace the body with:
    //
    //      #include <Arduino.h>
    //      const int ARM_PIN = 34;   // whatever GPIO you wired
    //
    //      bool isArmPinHigh() {
    //          return digitalRead(ARM_PIN) == HIGH;
    //      }
    //
    //  Wire the switch: 3.3V → switch → GPIO pin
    //                   GPIO pin → 10kΩ resistor → GND
    //  When switch is open  → pin reads LOW  → stays in WAIT_FOR_ARM
    //  When switch is closed → pin reads HIGH → countdown begins
    //
    bool isArmPinHigh() {
        return true;   // simulation: always armed
    }

    // ── WAIT_FOR_ARM ──────────────────────────────────────────
    //  Sit here until the physical arm switch is flipped.
    //  Prints a reminder every 3 seconds so you know it's alive.
    void runWaitForArm(DronePacket& cmd, double dt) {
        cmd.armed    = false;
        cmd.throttle = 0.0;

        _waitPrintTimer += dt;

        if (isArmPinHigh()) {
            std::cout << "\n[MISSION] Arm switch detected. Starting countdown.\n"
                      << "[MISSION] >>> STAND CLEAR OF THE DRONE <<<\n\n";
            _stage = Stage::COUNTDOWN;
        } else {
            // Print a waiting message every 3 seconds
            if (_waitPrintTimer >= 3.0) {
                std::cout << "[MISSION] Waiting for arm switch...\n";
                _waitPrintTimer = 0.0;
            }
        }
    }

    // ── COUNTDOWN ─────────────────────────────────────────────
    //  Counts down from LAUNCH_COUNTDOWN_SEC to 0.
    //  Prints one line per second so you can hear/see the countdown.
    //  If the arm switch is dropped during countdown, it resets.
    //  This gives you time to walk away safely.
    void runCountdown(DronePacket& cmd, double dt) {
        cmd.armed    = false;   // motors still OFF during countdown
        cmd.throttle = 0.0;

        // Safety: if arm switch dropped, reset the whole countdown
        if (!isArmPinHigh()) {
            std::cout << "\n[MISSION] Arm switch released — countdown RESET.\n\n";
            _countdownTimer = 0.0;
            _lastBeepSecond = -1;
            _stage = Stage::WAIT_FOR_ARM;
            return;
        }

        _countdownTimer += dt;

        // Print once per whole second
        int secondsElapsed  = (int)_countdownTimer;
        int secondsLeft     = (int)LAUNCH_COUNTDOWN_SEC - secondsElapsed;

        if (secondsElapsed != _lastBeepSecond) {
            _lastBeepSecond = secondsElapsed;
            if (secondsLeft > 0) {
                std::cout << "[COUNTDOWN] " << secondsLeft
                          << (secondsLeft == 1 ? " second..." : " seconds...")
                          << "\n";
            }
        }

        if (_countdownTimer >= LAUNCH_COUNTDOWN_SEC) {
            std::cout << "\n[MISSION] LAUNCH. Arming motors.\n\n";
            _stage = Stage::TAKEOFF;
        }
    }

    // ── TAKEOFF ───────────────────────────────────────────────
    void runTakeoff(DronePacket& cmd, double altitude, double roll, double pitch) {
        cmd.armed       = true;
        cmd.throttle    = 0.6;
        cmd.targetRoll  = 0.0;
        cmd.targetPitch = 0.0;

        if (altitude >= TARGET_ALTITUDE_M - 0.05) {
            std::cout << "\n[MISSION] Target altitude reached (" 
                      << altitude << "m). Entering hover.\n\n";
            _stage = Stage::HOVER;
        }
    }

    // ── HOVER ─────────────────────────────────────────────────
    void runHover(DronePacket& cmd, double altitude, double dt) {
        _hoverTimer += dt;
        cmd.armed       = true;
        cmd.throttle    = 0.15;  // Start lower — let altitude PID trim up as needed
        cmd.targetRoll  = 0.0;
        cmd.targetPitch = 0.0;

        if (_hoverTimer >= HOVER_DURATION_SEC) {
            std::cout << "\n[MISSION] Hover complete. Beginning descent.\n\n";
            _stage = Stage::LAND;
        }
    }

    // ── LAND ──────────────────────────────────────────────────
    void runLand(DronePacket& cmd, double altitude, double roll, double pitch) {
        // On first entry to LAND, freeze the current attitude
        static bool land_started = false;
        if (!land_started) {
            _landStartRoll  = roll;
            _landStartPitch = pitch;
            land_started = true;
        }
        
        cmd.armed       = true;
        cmd.throttle    = 0.05;  // Very low throttle — force descent
        cmd.targetRoll  = _landStartRoll;   // Hold landing attitude — no corrections
        cmd.targetPitch = _landStartPitch;  // prevents PID-driven unbalanced thrust

        if (altitude <= 0.05) {
            std::cout << "\n[MISSION] Touchdown. Disarming motors.\n\n";
            _stage = Stage::SHUTDOWN;
            land_started = false;  // reset for next flight
        }
    }

    // ── Utility: abs without <cmath> dependency ───────────────
    double absVal(double v) { return v < 0.0 ? -v : v; }
};