#pragma once
#include "../interfaces/IESC.hpp"

#ifdef ON_REAL_HARDWARE
#include "Arduino.h"
#include "driver/rmt.h"
#include <cstring>

// ============================================================
//  EspESC  —  Real DShot600 ESC driver for ESP32-S3
//
//  Sends DShot600 digital commands to four ESCs using the
//  ESP32-S3's RMT (Remote Control Module) peripheral.
//
//  DShot600 Protocol:
//    - 600 kbaud = 1.67 µs per bit
//    - 16-bit frame: 11-bit throttle + 1-bit arm + 4-bit CRC
//    - Throttle range: 0-2047 (0% to 100%)
//    - Arm bit: must be set for motor to spin
//    - CRC: XOR of throttle and arm bits
//    - Frame rate: 32 kHz (every 31.25 µs)
//
//  Advantages over PWM:
//    ✓ Digital = no calibration needed, immune to noise
//    ✓ Higher frame rate = smoother control
//    ✓ Bidirectional capable (ESCs can report telemetry)
//    ✓ Standard in modern racing drones
//
//  Wiring — connect each ESC signal wire to RMT-capable pin:
//
//         FRONT
//    M1 ──── GPIO 4  (RMT0)    M2 ──── GPIO 5  (RMT1)
//       \                          /
//        \                        /
//        /                        \
//       /                          \
//    M3 ──── GPIO 6  (RMT2)    M4 ──── GPIO 7  (RMT3)
//         REAR
//
//  Each ESC also needs:
//    - Power from your battery (via PDB or directly)
//    - Ground shared with the ESP32
//    - Signal wire to the GPIO pin above
//
//  Note: These are not standard RMT pins; normally RMT0-3
//  are on GPIO 0-3. If your board routes differently, edit
//  PIN_M* and gpio_num_t assignments below.
// ============================================================

// ── Pin assignments (GPIO → RMT channel) ──────────────────
static constexpr int PIN_M1 = 4;   // front-left  → RMT channel 0
static constexpr int PIN_M2 = 5;   // front-right → RMT channel 1
static constexpr int PIN_M3 = 6;   // rear-left   → RMT channel 2
static constexpr int PIN_M4 = 7;   // rear-right  → RMT channel 3

// ── DShot600 timing ────────────────────────────────────────
//
//  DShot600 = 600 kbaud → 1.67 µs per bit
//  Using 80 MHz clock on ESP32-S3:
//    clock_div = 80 MHz / 2 MHz (approx) = 40 → gives ~1.67 µs per tick
//    1 bit = 1.2 ticks ≈ 1.67 µs
//
static constexpr uint32_t DSHOT_FREQ = 600000;        // 600 kbaud
static constexpr uint32_t RMT_CLOCK_DIV = 40;         // 80MHz / 40 = 2MHz clock
static constexpr uint32_t TICKS_PER_BIT = 1;          // ~0.5µs per tick (conservative)
static constexpr uint32_t DSHOT_HIGH = 1;             // bit '1' = ~1.2µs high
static constexpr uint32_t DSHOT_LOW = 0;              // bit '0' = ~0.4µs high

class EspESC : public IESC {
public:
    void initialize() override {
        initRmtChannel(RMT_CHANNEL_0, (gpio_num_t)PIN_M1);
        initRmtChannel(RMT_CHANNEL_1, (gpio_num_t)PIN_M2);
        initRmtChannel(RMT_CHANNEL_2, (gpio_num_t)PIN_M3);
        initRmtChannel(RMT_CHANNEL_3, (gpio_num_t)PIN_M4);

        Serial.println("[ESC] EspESC (DShot600) initialised on GPIO 4/5/6/7");
        Serial.println("[ESC] Waiting for ESC arm sequence...");
        
        // Send disarm signal to all motors
        disarm();
        delay(1000);
        
        // Send neutral throttle command for 1 second (ESC arming handshake)
        for (int i = 0; i < 100; i++) {
            write(0.0, 0.0, 0.0, 0.0);  // 0% throttle for 10ms × 100 = 1s
            delay(10);
        }
        
        Serial.println("[ESC] ESC arm sequence complete!");
    }

    // write() is called by MotorMixer every 4ms (250Hz).
    // Values are 0.0 (stopped) to 1.0 (full throttle).
    void write(double m1, double m2, double m3, double m4) override {
        sendDshot(RMT_CHANNEL_0, m1);
        sendDshot(RMT_CHANNEL_1, m2);
        sendDshot(RMT_CHANNEL_2, m3);
        sendDshot(RMT_CHANNEL_3, m4);
    }

    // disarm() sends minimum throttle command to all motors.
    void disarm() override {
        write(0.0, 0.0, 0.0, 0.0);
    }

    // getMotor() returns the last commanded throttle value (0.0-1.0)
    // On real hardware, we track what we sent (no feedback from ESCs without telemetry)
    double getMotor(int index) const override {
        if (index < 1 || index > 4) return 0.0;
        return last_throttle[index - 1];
    }

private:
    // Track last commanded throttle for getMotor()
    // Mutable allows update in const methods
    mutable double last_throttle[4] = {0.0, 0.0, 0.0, 0.0};

    // Initialize one RMT channel for DShot600 output
    void initRmtChannel(rmt_channel_t channel, gpio_num_t gpio) {
        rmt_config_t rmt_cfg;
        rmt_cfg.rmt_mode = RMT_MODE_TX;
        rmt_cfg.channel = channel;
        rmt_cfg.gpio_num = gpio;
        rmt_cfg.mem_block_num = 1;
        rmt_cfg.clk_div = RMT_CLOCK_DIV;

        rmt_cfg.tx_config.loop_en = false;
        rmt_cfg.tx_config.carrier_en = false;
        rmt_cfg.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
        rmt_cfg.tx_config.carrier_duty_percent = 50;
        rmt_cfg.tx_config.carrier_freq_hz = 0;
        rmt_cfg.tx_config.carrier_level = RMT_CARRIER_LEVEL_LOW;

        ESP_ERROR_CHECK(rmt_config(&rmt_cfg));
        ESP_ERROR_CHECK(rmt_driver_install(channel, 0, 0));
    }

    // Convert throttle (0.0-1.0) to DShot600 command and transmit
    void sendDshot(rmt_channel_t channel, double throttle) {
        // Store for getMotor() readback
        last_throttle[channel] = throttle;

        // Clamp throttle to valid range
        if (throttle < 0.0) throttle = 0.0;
        if (throttle > 1.0) throttle = 1.0;

        // Convert 0.0-1.0 to 0-2047 DShot throttle value
        // DShot range: 0-47 = invalid/reserved, 48-2047 = throttle, 2048+ = special commands
        uint16_t throttle_value = (uint16_t)(48 + throttle * (2047 - 48));

        // Build 16-bit frame: [throttle(11) | arm(1) | checksum(4)]
        uint16_t armed_throttle = (throttle_value << 1) | 0x01;  // arm bit = 1
        uint8_t checksum = calcDshotChecksum(throttle_value, 1);
        uint16_t frame = (armed_throttle << 4) | checksum;

        // Encode frame as RMT pulses (16 bits)
        rmt_item32_t items[16];
        for (int i = 0; i < 16; i++) {
            uint8_t bit = (frame >> (15 - i)) & 0x01;
            
            if (bit) {
                // DShot600 '1' bit: ~1.2µs high, ~0.6µs low (total ~2µs = 1.2 ticks)
                items[i].level0 = 1;
                items[i].duration0 = 2;  // ~1.0µs
                items[i].level1 = 0;
                items[i].duration1 = 1;  // ~0.5µs
            } else {
                // DShot600 '0' bit: ~0.6µs high, ~1.2µs low (total ~2µs = 1.2 ticks)
                items[i].level0 = 1;
                items[i].duration0 = 1;  // ~0.5µs
                items[i].level1 = 0;
                items[i].duration1 = 2;  // ~1.0µs
            }
        }

        // Send RMT frame (blocking, ~30µs total)
        rmt_write_items(channel, items, 16, true);  // true = wait for TX
    }

    // DShot600 CRC: standard nibble XOR of throttle value
    // Formula: (throttle ^ (throttle >> 4) ^ (throttle >> 8)) & 0x0F
    // This creates a 4-bit checksum that ESCs use to validate the frame
    static uint8_t calcDshotChecksum(uint16_t throttle, uint8_t arm) {
        // CRC is computed from throttle only, not the arm bit
        uint8_t crc = (throttle ^ (throttle >> 4) ^ (throttle >> 8)) & 0x0F;
        return crc;
    }
};

#endif // ON_REAL_HARDWARE