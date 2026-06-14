#pragma once

// ============================================================
//  IPlatformMutex
//
//  Wraps a mutex so FlightControllerProgram has no idea
//  whether it's running std::mutex (Mac) or a FreeRTOS
//  semaphore (ESP32-S3).
// ============================================================
class IPlatformMutex {
public:
    virtual void lock()   = 0;
    virtual void unlock() = 0;
    virtual ~IPlatformMutex() = default;
};

// RAII guard — works exactly like std::lock_guard
class PlatformLockGuard {
public:
    explicit PlatformLockGuard(IPlatformMutex& m) : mutex(m) { mutex.lock(); }
    ~PlatformLockGuard() { mutex.unlock(); }
    PlatformLockGuard(const PlatformLockGuard&) = delete;
private:
    IPlatformMutex& mutex;
};