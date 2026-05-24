#pragma once
#include "IPlatformMutex.hpp"

#ifdef ESP_BUILD
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// ============================================================
//  EspMutex  (ESP32-S3 hardware)
//
//  Wraps a FreeRTOS mutex semaphore. Must be created before
//  the scheduler starts — construct these in main() before
//  calling program.setup().
// ============================================================
class EspMutex : public IPlatformMutex {
public:
    EspMutex()  { handle = xSemaphoreCreateMutex(); }
    ~EspMutex() { vSemaphoreDelete(handle); }

    void lock()   override { xSemaphoreTake(handle, portMAX_DELAY); }
    void unlock() override { xSemaphoreGive(handle); }
private:
    SemaphoreHandle_t handle;
};

#endif // ESP_BUILD