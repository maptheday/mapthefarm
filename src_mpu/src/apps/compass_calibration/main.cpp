// ============================================================================
// APP: compass_calibration -- teach the compass this drone's magnetic quirks.
// Do it ONCE (and again if you add or move metal/wiring near the compass).
// Motors stay OFF.
//
//   flash:  pio run -e compass_calibration -t upload
//   then:   take the drone OUTDOORS, away from cars and metal, power it up,
//           open the serial monitor (115200), and for 30 seconds slowly rotate
//           it through every orientation (every side facing down, then spin it
//           around). The result is saved on the drone and survives reflashing.
//   after:  flash the fly app again (pio run -e fly -t upload)
// ============================================================================

#include "flight/FlightController.hpp"
#include "flight/hardware/HardwareIo.hpp"

HardwareIo hardware;
bool calibrating = false;

void setup() {
  Serial.begin(115200);
  delay(500);

  String errors;
  if (!fc::loadSettings(errors)) fc::halt("Can't load flight settings:\n" + errors);
  // The real sensors and ESCs, but no radio plugged in: this app must never
  // take off.
  FlightIo io = hardware.io();
  io.radio = nullptr;
  fc::begin(io);

  Serial.println("=== COMPASS CALIBRATION ===");
  Serial.println("Starting in 3 seconds: pick the drone up (motors stay off).");
  delay(3000);
  fc::calibrateCompass();   // runs the CALIBRATE phase, then returns to PARKED
  calibrating = true;
}

void loop() {
  if (calibrating && fc::phase() == PHASE_PARKED) {
    calibrating = false;
    Serial.println("=== DONE: calibration saved. Flash the fly app again. ===");
  }
  delay(200);
}
