#pragma once
#include "../interfaces/IPlatformLauncher.hpp"
#include <functional>

// ============================================================
//  EspLauncher  (ESP32-S3 hardware)
//
//  Pins each core function to its physical core via FreeRTOS
//  xTaskCreatePinnedToCore(). This matches how Cortex itself
//  runs its radio, avionics, and blackbox tasks on Core 0,
//  and the flight loop on Core 1.
//
//  Stack size of 8192 is conservative — tune down if RAM is
//  tight, or up if you add more local state to the core loops.
//
//  Usage:
//    EspLauncher launcher;
//    program.runDualCore(launcher);
//
//  Only compiled when ESP_BUILD is defined (set via
//  build_flags in platformio.ini for the esp32s3 env).
// ============================================================

#ifdef ESP_BUILD

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// FreeRTOS task entry points need plain function pointers,
// so we store the std::function targets in file-scope globals.
static std::function<void()> g_esp_core0_fn;
static std::function<void()> g_esp_core1_fn;

static void esp_core0_task(void*) {
    g_esp_core0_fn();
    vTaskDelete(nullptr);
}

static void esp_core1_task(void*) {
    g_esp_core1_fn();
    vTaskDelete(nullptr);
}

class EspLauncher : public IPlatformLauncher {
public:
    void launchCore0(std::function<void()> fn) override {
        g_esp_core0_fn = fn;
        xTaskCreatePinnedToCore(
            esp_core0_task,   // entry point
            "avionics_core0", // task name (shows in pio monitor)
            8192,             // stack size in bytes
            nullptr,          // parameter
            1,                // priority
            nullptr,          // task handle (not needed)
            0                 // pin to Core 0
        );
    }

    void launchCore1(std::function<void()> fn) override {
        g_esp_core1_fn = fn;
        xTaskCreatePinnedToCore(
            esp_core1_task,
            "flight_core1",
            8192,
            nullptr,
            1,
            nullptr,
            1                 // pin to Core 1
        );
    }

    void waitForCompletion() override {
        // FreeRTOS scheduler owns execution from here.
        // main() must not return — delay forever.
        vTaskDelay(portMAX_DELAY);
    }
};

#endif // ESP_BUILD