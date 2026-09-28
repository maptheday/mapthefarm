// ============================================================================
// APP: fly -- the real flight program.
//
//   flash:     pio run -e fly -t upload
//   settings:  pio run -e fly -t uploadfs      (puts data/flightsettings.json on the drone)
//
// Loads the settings, connects the real sensors and ESCs, and hands control to
// the radio: START takes off / starts the mission, LAND lands, STOP cuts the
// motors, MANUAL hands over the sticks. The mission is the route in
// flightsettings.json ("mission.route").
// ============================================================================

#include "flight/FlightController.hpp"
#include "flight/io/HardwareIo.hpp"

HardwareIo hardware;

void setup() {
  Serial.begin(115200);
  delay(500);

  String errors;
  if (!fc::loadSettings(errors)) fc::halt("Can't load flight settings:\n" + errors);

  fc::begin(hardware);   // sensors, ESCs, radio, then the flight loops
}

void loop() {
  // Everything runs in the flight controller's own tasks.
  vTaskDelay(pdMS_TO_TICKS(1000));
}
