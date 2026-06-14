#pragma once
#include <functional>

// ============================================================
//  IPlatformLauncher
//
//  Abstract interface for launching the two flight controller
//  cores. Swap implementations to target different platforms
//  without touching FlightControllerProgram at all.
//
//  Implementations:
//    ThreadLauncher  — Mac/Linux simulation (std::thread)
//    EspLauncher     — ESP32-S3 hardware (FreeRTOS)
// ============================================================
class IPlatformLauncher {
public:
    virtual void launchCore0(std::function<void()> fn) = 0;
    virtual void launchCore1(std::function<void()> fn) = 0;
    virtual void waitForCompletion() = 0;
    virtual ~IPlatformLauncher() = default;
};