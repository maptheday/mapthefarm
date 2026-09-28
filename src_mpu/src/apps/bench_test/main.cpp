// ============================================================================
// APP: bench_test -- a PROPS-OFF checkout of the freshly built drone.
//
//   flash:  pio run -e bench_test -t upload
//   then:   open the serial monitor (115200) and type commands:
//             1..4  spin ONE motor slowly for 2 seconds -- check it's the right
//                   corner and spinning the right direction (see IESC.hpp)
//             s     stream every sensor for 20 seconds -- tilt the drone by hand
//                   and check the numbers move the right way; also read the
//                   battery voltage, to compare against a multimeter
//   after:  flash the fly app again (pio run -e fly -t upload)
//
// Uses the real sensors and ESCs (HardwareIo) but never starts the flight
// loops or the radio, so the drone can't fly while this app is on it.
// ============================================================================

#include "flight/FlightController.hpp"
#include "flight/io/HardwareIo.hpp"
#include "../common/SerialInput.hpp"

HardwareIo hardware;

// Slow enough to be safe on the bench, fast enough that every ESC starts the motor.
static const float BENCH_MOTOR_THROTTLE = 0.12f;

void printMenu() {
  Serial.println();
  Serial.println("=== BENCH TEST (PROPS OFF!) ===");
  Serial.println("  1 2 3 4   spin that motor slowly for 2 s");
  Serial.println("            (1 = front-left, 2 = front-right, 3 = rear-left, 4 = rear-right)");
  Serial.println("  s         stream sensor readings for 20 s (tilt the drone and watch)");
  Serial.println("Type a command and press Enter.");
}

void spinMotor(int motor) {
  const char* names[4] = { "FRONT-LEFT", "FRONT-RIGHT", "REAR-LEFT", "REAR-RIGHT" };
  const char* dirs[4]  = { "CLOCKWISE", "COUNTER-CLOCKWISE", "COUNTER-CLOCKWISE", "CLOCKWISE" };
  Serial.println(String("Spinning M") + motor + " (" + names[motor - 1] + ") for 2 s. "
                 "Seen from ABOVE it should spin " + dirs[motor - 1] + ".");
  hardware.writeMotor(motor, BENCH_MOTOR_THROTTLE);
  delay(2000);
  hardware.stopMotor(motor);
  Serial.println("Stopped. Wrong corner? Move that ESC's signal wire. "
                 "Wrong direction? Swap any two of that motor's three wires.");
}

void streamSensors() {
  Serial.println("Streaming for 20 s. Try: tilt nose UP (pitch should go +), tilt RIGHT side down "
                 "(roll should go +), turn clockwise by hand (heading goes up, yaw rate +).");
  unsigned long start = millis(), lastPrint = 0, lastMicros = micros();
  while (millis() - start < 20000) {
    unsigned long now = micros();
    float dt = (now - lastMicros) / 1000000.0f;
    lastMicros = now;

    FastInputs fast;
    hardware.readFast(dt, fast);                      // keeps the attitude filter fed
    battery.update(fast.packVolts, MotorMix{}, dt);   // motors off: idle current only

    if (millis() - lastPrint >= 500) {
      lastPrint = millis();
      SlowInputs slow;
      hardware.readSlow(slow);
      const RawBattery& b = battery.reading();
      Serial.println("roll " + String(fast.imu.gyroX, 1) +
                     "  pitch " + String(fast.imu.gyroY, 1) +
                     "  yawRate " + String(fast.imu.yawRateDps, 0) + "/s" +
                     "  | heading " + String(slow.compassHeadingDeg, 0) +
                     "  | height " + String(fast.baroAltitudeFt, 1) + " ft" +
                     "  | GPS " + (slow.gps.fix ? String(slow.gps.lat, 6) + ", " + String(slow.gps.lon, 6)
                                                : String("no fix")) +
                     " (" + slow.gps.sats + " sats)" +
                     "  | battery " + (b.present ? String(b.packVolts, 2) + " V (" +
                                                   String(b.cellVolts, 2) + " V/cell)"
                                                 : String("not detected")));
    }
    delay(10);
  }
  Serial.println("Done streaming.");
}

void setup() {
  Serial.begin(115200);
  delay(500);

  String errors;
  if (!fc::loadSettings(errors)) fc::halt("Can't load flight settings:\n" + errors);
  hardware.beginMotors();    // the ESCs (stopped)
  hardware.beginSensors();   // IMU, compass, GPS, barometer, battery pin -- no radio
}

void loop() {
  printMenu();
  String cmd = readSerialLine();
  if (cmd == "1" || cmd == "2" || cmd == "3" || cmd == "4") spinMotor(cmd.toInt());
  else if (cmd.equalsIgnoreCase("s")) streamSensors();
  else Serial.println("Unknown command: " + cmd);
}
