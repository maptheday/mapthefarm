#pragma once

// ============================================================================
// BENCH TEST -- a props-OFF checkout mode for the freshly built drone.
// Turned on with BENCH_TEST_ON_BOOT in FlightConfig.hpp. Runs from setup()
// before the flight tasks start, and never returns, so the drone can't fly
// while it's on.
//
// Over the serial monitor (115200) it offers:
//   1..4  spin ONE motor slowly for 2 seconds -- check it's the right corner
//         and spinning the right direction (see the diagram in IESC.hpp)
//   s     stream every sensor for 20 seconds -- tilt the drone by hand and
//         check the numbers move the right way
// Real hardware only (no motors or sensors exist in a SIM build).
// ============================================================================

#include <Arduino.h>
#include "../models/SensorTypes.hpp"   // RawImuReading, RawGpsReading
#include "Log.hpp"                     // logLine
#include "Motors.hpp"                  // motors
#include "Imu.hpp"                     // imu
#include "Compass.hpp"                 // compass
#include "Altimeter.hpp"               // altimeter
#include "Gps.hpp"                     // gps

#ifndef SIM

// Slow enough to be safe on the bench, fast enough that every ESC starts the motor.
static const float BENCH_MOTOR_THROTTLE = 0.12f;

// Read one line typed into the serial monitor (blocks until Enter).
inline String benchReadLine() {
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

inline void benchPrintMenu() {
  logLine("");
  logLine("=== BENCH TEST (PROPS OFF!) ===");
  logLine("  1 2 3 4   spin that motor slowly for 2 s");
  logLine("            (1 = front-left, 2 = front-right, 3 = rear-left, 4 = rear-right)");
  logLine("  s         stream sensor readings for 20 s (tilt the drone and watch)");
  logLine("Type a command and press Enter.");
}

inline void benchSpinMotor(int motor) {
  const char* names[4] = { "FRONT-LEFT", "FRONT-RIGHT", "REAR-LEFT", "REAR-RIGHT" };
  const char* dirs[4]  = { "CLOCKWISE", "COUNTER-CLOCKWISE", "COUNTER-CLOCKWISE", "CLOCKWISE" };
  logLine(String("Spinning M") + motor + " (" + names[motor - 1] + ") for 2 s. "
          "Seen from ABOVE it should spin " + dirs[motor - 1] + ".");
  motors.writeOne(motor, BENCH_MOTOR_THROTTLE);
  delay(2000);
  motors.disarmOne(motor);
  logLine("Stopped. Wrong corner? Move that ESC's signal wire. "
          "Wrong direction? Swap any two of that motor's three wires.");
}

inline void benchStreamSensors() {
  logLine("Streaming for 20 s. Try: tilt nose UP (pitch should go +), tilt RIGHT side down "
          "(roll should go +), turn clockwise by hand (heading goes up, yaw rate +).");
  RawImuReading imuReading;
  RawGpsReading gpsReading;
  unsigned long start = millis(), lastPrint = 0, lastMicros = micros();
  while (millis() - start < 20000) {
    unsigned long now = micros();
    float dt = (now - lastMicros) / 1000000.0f;
    lastMicros = now;
    imu.read(imuReading, dt);         // keep the attitude filter fed at ~100 Hz
    gps.read(gpsReading);             // keep the GPS parser fed
    if (millis() - lastPrint >= 500) {
      lastPrint = millis();
      logLine("roll " + String(imuReading.gyroX, 1) +
              "  pitch " + String(imuReading.gyroY, 1) +
              "  yawRate " + String(imuReading.yawRateDps, 0) + "/s" +
              "  | heading " + String(compass.readHeadingDeg(), 0) +
              "  | height " + String(altimeter.readAltitudeFt(), 1) + " ft" +
              "  | GPS " + (gpsReading.fix ? String(gpsReading.lat, 6) + ", " + String(gpsReading.lon, 6)
                                           : String("no fix")) +
              " (" + gpsReading.sats + " sats)");
    }
    delay(10);
  }
  logLine("Done streaming.");
}

// Never returns: the drone stays on the bench until you turn the flag off.
inline void runBenchTest() {
  motors.disarmAll();
  while (true) {
    benchPrintMenu();
    String cmd = benchReadLine();
    if (cmd == "1" || cmd == "2" || cmd == "3" || cmd == "4") benchSpinMotor(cmd.toInt());
    else if (cmd.equalsIgnoreCase("s")) benchStreamSensors();
    else logLine("Unknown command: " + cmd);
  }
}

#endif  // SIM
