#pragma once
#include "IPlatformLauncher.hpp"
#include <thread>

// ============================================================
//  ThreadLauncher  (Mac / Linux simulation)
//
//  Runs Core 0 and Core 1 as two std::threads. Used when
//  building and testing on your Mac — no hardware needed.
//
//  Usage:
//    ThreadLauncher launcher;
//    program.runDualCore(launcher);
// ============================================================
class ThreadLauncher : public IPlatformLauncher {
public:
    void launchCore0(std::function<void()> fn) override {
        t0 = std::thread(fn);
    }

    void launchCore1(std::function<void()> fn) override {
        t1 = std::thread(fn);
    }

    void waitForCompletion() override {
        if (t0.joinable()) t0.join();
        if (t1.joinable()) t1.join();
    }

private:
    std::thread t0;
    std::thread t1;
};