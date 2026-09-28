// ============================================================================
// APP: first_mission -- the real flight program, with a SHORT mission.
//
//   flash:  pio run -e first_mission -t upload
//
// Exactly like the fly app (radio START / LAND / STOP / MANUAL all work the
// same), except the mission is a 30 m out-and-back along the first leg of the
// route in flightsettings.json, at 20 ft (see apps/common/TestRoute.hpp).
// Launch from the start of the route (corner 1). Fly this until you're
// confident, then flash the fly app for the full route.
// ============================================================================

#include "flight/FlightController.hpp"
#include "flight/io/HardwareIo.hpp"
#include "../common/TestRoute.hpp"

HardwareIo hardware;

void setup() {
  Serial.begin(115200);
  delay(500);

  String errors;
  if (!fc::loadSettings(errors)) fc::halt("Can't load flight settings:\n" + errors);

  fc::begin(hardware);
  fc::setMission(makeTestRoute(settings().mission.route));
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
