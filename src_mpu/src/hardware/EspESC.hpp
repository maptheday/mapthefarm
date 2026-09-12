#pragma once
#include <Arduino.h>
#include "driver/rmt.h"

// ============================================================
//  EspESC  —  DShot600 ESC driver for ESP32-S3
//
//  One instance per motor. Call init() with the GPIO pin
//  and RMT channel, then sendThrottle() each loop.
//
//  DShot600 Protocol:
//    - 16-bit frame: 11-bit value + 1-bit telemetry + 4-bit CRC
//    - Throttle range: 48-2047 (0-47 are reserved commands)
//    - '1' bit = 190 ticks high, 76 ticks low
//    - '0' bit =  95 ticks high, 171 ticks low
//
//  Wiring:
//         FRONT
//    M1 ── GPIO 4    M2 ── GPIO 5
//    M3 ── GPIO 6    M4 ── GPIO 7
//         REAR
// ============================================================

class EspESC {
public:
    // Call once in setup() — sets up the RMT channel for this motor
    void init(int pin, rmt_channel_t channel) {
        _channel = channel;

        // RMT_DEFAULT_CONFIG_TX fills all the boilerplate fields for us
        rmt_config_t config = RMT_DEFAULT_CONFIG_TX((gpio_num_t)pin, channel);
        config.clk_div = 1; // full 80MHz clock for precise DShot timing

        rmt_config(&config);
        rmt_driver_install(config.channel, 0, 0);
    }

    // Send a throttle value (0.0 - 1.0)
    // Converts to DShot range 48-2047 and transmits
    void write(float throttle) {
        if (throttle < 0.0f) throttle = 0.0f;
        if (throttle > 1.0f) throttle = 1.0f;

        // 0.0 → 48, 1.0 → 2047
        uint16_t value = (uint16_t)(48 + throttle * (2047 - 48));
        sendThrottle(value);
    }

    // Cut this motor immediately
    void disarm() {
        sendThrottle(0);
    }

private:
    rmt_channel_t _channel;
    rmt_item32_t  _items[17]; // 16 bits + 1 terminator

    void sendThrottle(uint16_t throttle) {
        if (throttle > 2047) throttle = 2047;
        if (throttle > 0 && throttle < 48) throttle = 48; // safety: skip reserved range
        sendPacket(throttle);
    }

    void sendPacket(uint16_t value) {
        // Build 16-bit frame: [value(11) | telemetry(1) | CRC(4)]
        uint16_t packet = (value << 1) | 0; // telemetry bit = 0

        // CRC: XOR each nibble together
        int csum = 0;
        int csum_data = packet;
        for (int i = 0; i < 3; i++) {
            csum ^= (csum_data & 0x0F);
            csum_data >>= 4;
        }
        packet = (packet << 4) | (csum & 0x0F);

        // Encode 16 bits as RMT pulses
        // DShot600 '1' bit: 190 ticks high, 76 ticks low
        // DShot600 '0' bit:  95 ticks high, 171 ticks low
        for (int i = 0; i < 16; i++) {
            bool bit = (packet & 0x8000);
            packet <<= 1;
            if (bit) { _items[i] = {{{ 190, 1,  76, 0 }}}; }
            else     { _items[i] = {{{  95, 1, 171, 0 }}}; }
        }
        _items[16] = {{{ 0, 0, 0, 0 }}}; // terminator

        // Non-blocking send — DShot doesn't need to wait for TX to finish
        rmt_write_items(_channel, _items, 16, false);
    }
};