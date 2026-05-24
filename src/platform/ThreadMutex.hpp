#pragma once
#include "IPlatformMutex.hpp"
#include <mutex>

// ============================================================
//  ThreadMutex  (Mac / Linux simulation)
//
//  Wraps std::mutex for use in the native simulation env.
// ============================================================
class ThreadMutex : public IPlatformMutex {
public:
    void lock()   override { m.lock(); }
    void unlock() override { m.unlock(); }
private:
    std::mutex m;
};