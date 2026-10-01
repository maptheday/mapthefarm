#pragma once
#include <Arduino.h>

// ============================================================
//  EspPwmESC  —  standard PWM ESC driver for ESP32-S3
//
//  One instance per motor. Call init() with the GPIO pin and a
//  PWM channel, then write() a throttle each loop.
//
//  PWM is the classic "servo" signal every ESC understands:
//  a steady stream of pulses, where the pulse WIDTH is the
//  throttle.
//
//     1000 us high  = stopped
//     1500 us high  = half throttle
//     2000 us high  = full throttle
//
//         ┌──┐          ┌────┐          ┌──────┐
//     ────┘  └──────────┘    └──────────┘      └────
//        1000us        1500us          2000us
//        (stop)        (half)          (full)
//
//  The pulses repeat `hz` times a second (400 = one every
//  2500 us). The ESP32's LEDC hardware generates them on its own,
//  so the CPU just sets the width and moves on.
//
//  PWM ESCs must be calibrated once so they learn that 1000 us
//  means stop and 2000 us means full: that's the esc_calibration app
//  (src/apps/esc_calibration).
//
//  Wiring:
//         FRONT
//    M1 ── GPIO 4    M2 ── GPIO 5
//    M3 ── GPIO 6    M4 ── GPIO 7
//         REAR
// ============================================================

class EspPwmESC {
public:
    static const int MIN_US = 1000;   // stopped
    static const int MAX_US = 2000;   // full throttle

    // Call once in setup() -- sets up this motor's PWM channel (0-7), sending
    // `hz` pulses a second (wiring.escPwmHz in flightsettings.json).
    void init(int pin, int channel, int hz) {
        _channel = channel;
        _hz      = hz;
        ledcSetup(channel, hz, RESOLUTION_BITS);
        ledcAttachPin(pin, channel);
        disarm();
    }

    // Send a throttle value (0.0 - 1.0).
    // 0.0 -> 1000 us, 1.0 -> 2000 us.
    void write(float throttle) {
        if (throttle < 0.0f) throttle = 0.0f;
        if (throttle > 1.0f) throttle = 1.0f;
        writeMicroseconds(MIN_US + (int)(throttle * (MAX_US - MIN_US)));
    }

    // Stop this motor: the "stopped" pulse width.
    void disarm() {
        writeMicroseconds(MIN_US);
    }

    // Set the raw pulse width. Used directly only by ESC calibration.
    void writeMicroseconds(int us) {
        // The hardware counts in "duty" steps, not microseconds. One full
        // period (2500 us at 400 Hz) is MAX_DUTY steps, so scale us into steps.
        const uint32_t periodUs = 1000000UL / _hz;
        const uint32_t MAX_DUTY = (1UL << RESOLUTION_BITS) - 1;
        ledcWrite(_channel, (uint32_t)us * MAX_DUTY / periodUs);
    }

private:
    static const int RESOLUTION_BITS = 14;  // 16384 steps per period (~0.15 us each at 400 Hz)
    int _channel = 0;
    int _hz      = 400;
};
