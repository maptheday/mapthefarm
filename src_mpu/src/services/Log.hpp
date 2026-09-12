#pragma once

// ============================================================================
// LOG service -- thread-safe serial logging.
// The nav / physics / CRSF tasks all print from different cores; this holds a
// mutex so two lines never tear together on the wire. Under WOKWI_SIM these
// exact strings are also the HIL protocol, so keep messages verbatim.
// ============================================================================

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Created in setup(); defined once in the .ino.
extern SemaphoreHandle_t serialMutex;

inline void logLine(const String& msg) {
  if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    Serial.println(msg);
    Serial.flush(); // block until the line is actually on the wire -- USB CDC
                    // buffers writes, so without this a second task could start
                    // writing mid-drain and tear the two messages together.
    xSemaphoreGive(serialMutex);
  }
}
