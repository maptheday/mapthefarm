// ============================================================================
// APP: esc_calibration -- teach the 4 ESCs the throttle range. Do it ONCE,
// when the ESCs are new. PROPS OFF.
//
//   flash:  pio run -e esc_calibration -t upload
//   then:   open the serial monitor (115200) and follow the prompts
//   after:  flash the fly app again (pio run -e fly -t upload)
//
// Why this order: an ESC checks the signal the moment it gets battery power.
// If it sees FULL throttle then, it enters "learn the range" mode, beeps, and
// remembers that pulse width as full. Dropping to MIN then teaches it "stopped".
// ============================================================================

#include "flight/FlightController.hpp"
#include "flight/hardware/HardwareIo.hpp"
#include "../common/SerialInput.hpp"

PwmMotors* escs = new PwmMotors();   // only the ESCs: nothing else is needed

void setup() {
  Serial.begin(115200);
  delay(500);

  String errors;
  if (!fc::loadSettings(errors)) fc::halt("Can't load flight settings:\n" + errors);
  escs->begin();

  Serial.println();
  Serial.println("=== ESC CALIBRATION ===");
  Serial.println("1. Take ALL PROPS OFF.");
  Serial.println("2. UNPLUG the flight battery (the ESP32 stays powered by USB).");
  Serial.println("Type GO and press Enter when both are done.");
  waitForWord("GO");

  escs->writeAllMicroseconds(EspPwmESC::MAX_US);
  Serial.println("Sending FULL throttle signal (no power reaches the motors yet).");
  Serial.println("3. Plug in the flight battery now.");
  Serial.println("   The ESCs play a startup tune, then a short 'beep-beep'.");
  Serial.println("   Right after the beep-beep, type MIN and press Enter.");
  Serial.println("   (If the motors SPIN instead of beeping, unplug the battery at once.)");
  waitForWord("MIN");

  escs->writeAllMicroseconds(EspPwmESC::MIN_US);
  Serial.println("Sending STOPPED signal.");
  Serial.println("4. The ESCs beep once per battery cell (3 for 3S), then a long beep:");
  Serial.println("   the range is saved.");
  Serial.println("5. Unplug the battery, then flash the fly app again.");
  Serial.println("=== DONE (motors stay stopped) ===");
}

void loop() {
  delay(1000);   // nothing more to do; the ESCs keep receiving "stopped"
}
