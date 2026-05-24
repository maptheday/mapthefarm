#pragma once
#include "../interfaces/IESC.hpp"

#ifdef ESP_BUILD
#include "Arduino.h"

// ============================================================
//  EspESC  —  Real ESC driver for ESP32-S3
//
//  Sends PWM signals to four ESCs using the ESP32's built-in
//  LEDC (LED Control) peripheral, which is what the ESP32
//  uses for any PWM output — motors, servos, LEDs, anything.
//
//  Standard ESC protocol:
//    1000µs pulse = motor stopped
//    2000µs pulse = motor at full throttle
//    Signal repeats at 50Hz (every 20ms)
//
//  Wiring — connect each ESC signal wire to these pins:
//
//         FRONT
//    M1 ──── GPIO 4      M2 ──── GPIO 5
//       \                   /
//        \                 /
//        /                 \
//       /                   \
//    M3 ──── GPIO 6      M4 ──── GPIO 7
//         REAR
//
//  Each ESC also needs:
//    - Power from your battery (via PDB or directly)
//    - Ground shared with the ESP32
//    - Signal wire to the GPIO pin above
//
//  To change pins, edit the four PIN_M* constants below.
// ============================================================

// ── Pin assignments ───────────────────────────────────────
static constexpr int PIN_M1 = 4;   // front-left
static constexpr int PIN_M2 = 5;   // front-right
static constexpr int PIN_M3 = 6;   // rear-left
static constexpr int PIN_M4 = 7;   // rear-right

// ── LEDC configuration ────────────────────────────────────
//
//  LEDC works by counting up to a resolution value at a set
//  frequency. We use:
//    - 50 Hz:    standard ESC signal frequency
//    - 16-bit:   65535 steps, giving fine-grained control
//
//  At 50Hz and 16-bit resolution, the pulse width maps like:
//    1000µs (stopped)     = duty count 3277
//    2000µs (full thrust) = duty count 6554
//
//  Formula: duty = (pulse_us / 20000us) * 65535
//
static constexpr int    LEDC_FREQ_HZ    = 50;
static constexpr int    LEDC_RESOLUTION = 16;        // bits
static constexpr double LEDC_MAX_DUTY   = 65535.0;   // 2^16 - 1

// Pulse width in microseconds for stopped and full throttle
static constexpr double PULSE_MIN_US = 1000.0;
static constexpr double PULSE_MAX_US = 2000.0;
static constexpr double PERIOD_US    = 20000.0;      // 1/50Hz

class EspESC : public IESC {
public:
    void initialize() override {
        // Assign each motor pin to its own LEDC channel.
        // The ESP32-S3 has 8 channels — we use 0-3.
        ledcSetup(CHANNEL_M1, LEDC_FREQ_HZ, LEDC_RESOLUTION);
        ledcSetup(CHANNEL_M2, LEDC_FREQ_HZ, LEDC_RESOLUTION);
        ledcSetup(CHANNEL_M3, LEDC_FREQ_HZ, LEDC_RESOLUTION);
        ledcSetup(CHANNEL_M4, LEDC_FREQ_HZ, LEDC_RESOLUTION);

        ledcAttachPin(PIN_M1, CHANNEL_M1);
        ledcAttachPin(PIN_M2, CHANNEL_M2);
        ledcAttachPin(PIN_M3, CHANNEL_M3);
        ledcAttachPin(PIN_M4, CHANNEL_M4);

        // Send 1000µs (stopped) to all ESCs.
        // Most ESCs require seeing this signal on power-up
        // before they will arm and accept throttle commands.
        disarm();

        Serial.println("[ESC] EspESC initialised on pins 4/5/6/7");
        Serial.println("[ESC] Sending 1000us arm signal — wait 2s for ESC beeps");
        delay(2000);
    }

    // write() is called by MotorMixer every 4ms (250Hz).
    // Values are 0.0 (stopped) to 1.0 (full throttle).
    void write(double m1, double m2, double m3, double m4) override {
        ledcWrite(CHANNEL_M1, toDuty(m1));
        ledcWrite(CHANNEL_M2, toDuty(m2));
        ledcWrite(CHANNEL_M3, toDuty(m3));
        ledcWrite(CHANNEL_M4, toDuty(m4));
    }

    // disarm() sends the minimum pulse to all motors.
    // Call this on landing or any fault condition.
    void disarm() override {
        uint32_t stopped = toDuty(0.0);
        ledcWrite(CHANNEL_M1, stopped);
        ledcWrite(CHANNEL_M2, stopped);
        ledcWrite(CHANNEL_M3, stopped);
        ledcWrite(CHANNEL_M4, stopped);
    }

private:
    // LEDC channel numbers — one per motor
    static constexpr int CHANNEL_M1 = 0;
    static constexpr int CHANNEL_M2 = 1;
    static constexpr int CHANNEL_M3 = 2;
    static constexpr int CHANNEL_M4 = 3;

    // Convert a 0.0–1.0 throttle value into a LEDC duty count.
    //
    //  throttle 0.0 → 1000µs pulse → duty 3277
    //  throttle 1.0 → 2000µs pulse → duty 6554
    //
    static uint32_t toDuty(double throttle) {
        // Clamp input to safe range
        if (throttle < 0.0) throttle = 0.0;
        if (throttle > 1.0) throttle = 1.0;

        double pulse_us = PULSE_MIN_US + throttle * (PULSE_MAX_US - PULSE_MIN_US);
        double duty     = (pulse_us / PERIOD_US) * LEDC_MAX_DUTY;
        return static_cast<uint32_t>(duty);
    }
};

#endif // ESP_BUILD