// ============================================================================
// APP: first_mission -- the real flight program, with a SHORT mission.
//
//   flash:     pio run -e first_mission -t upload
//   settings:  pio run -e first_mission -t uploadfs
//
// Exactly like the fly app (radio START / LAND / STOP / MANUAL all work the
// same), except it layers data/flightsettings/first_mission.json on top of the
// normal settings -- like appsettings.Development.json in .NET. That file
// replaces the mission with a 30 m out-and-back along the first leg of the
// field route, at 20 ft. Launch from corner 1. Fly this until you're
// confident, then flash the fly app for the full route.
// ============================================================================

#include "flight/FlightController.hpp"
#include "flight/hardware/HardwareIo.hpp"

void setup() {
  Serial.begin(115200);
  delay(500);

  String errors;
  if (!fc::loadSettings(errors, {"first_mission"}))
    fc::halt("Can't load flight settings:\n" + errors);

  fc::begin(realHardware());
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
