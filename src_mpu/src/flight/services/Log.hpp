#pragma once

// ============================================================================
// LOG service -- thread-safe serial logging.
// The nav / physics / CRSF tasks all print from different cores; this holds a
// mutex so two lines never tear together on the wire. In the sim app these
// exact strings are also what the laptop scripts watch for, so keep them verbatim.
// ============================================================================

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Created by fc::loadSettings()/fc::begin(); defined once in FlightController.hpp.
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

// Unrecoverable-bug halt: print once and freeze so a broken drone never flies.
#define PANIC(msg) do { Serial.println(F("[PANIC] " msg " — halting")); while(1) { delay(10); } } while(0)
