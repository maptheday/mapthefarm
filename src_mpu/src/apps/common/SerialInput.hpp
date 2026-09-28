#pragma once

// Read what you type into the serial monitor (115200), one line at a time.
// Shared by the bench-side apps (bench_test, esc_calibration).

#include <Arduino.h>

// Block until a non-empty line arrives, then return it (trimmed).
inline String readSerialLine() {
  String line;
  while (true) {
    while (Serial.available()) {
      char c = (char)Serial.read();
      if (c == '\n' || c == '\r') {
        line.trim();
        if (line.length() > 0) return line;
      } else {
        line += c;
      }
    }
    delay(10);
  }
}

// Block until the given word is typed (not case-sensitive).
inline void waitForWord(const char* word) {
  while (true) {
    String line = readSerialLine();
    if (line.equalsIgnoreCase(word)) return;
    Serial.println(String("   (waiting for ") + word + ")");
  }
}
