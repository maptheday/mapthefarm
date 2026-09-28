// ============================================================================
// APP: sim -- the on-chip simulator. The REAL flight controller flies against
// simulated physics (SimIo), all on the ESP, at the real 200 Hz rate.
//
//   flash:     pio run -e sim -t upload
//   settings:  pio run -e sim -t uploadfs        (the same data/flightsettings.json)
//   run:       ./simulate/run_hil.sh             (flashes, runs every scenario, checks them)
//
// Talks to the laptop over USB:
//   SCENARIO:<name>  pick the scenario (sent right after reset, before takeoff):
//                    full / geofence / gpsloss / timeout / rcloss / lowbatt /
//                    land / testroute / manual / stab   (default: full)
//   DUMPLOG          stream the recorded flight log back
//   PING:            liveness check
// ============================================================================

#include "flight/FlightController.hpp"
#include "SimIo.hpp"
#include "../common/TestRoute.hpp"

SimIo sim;

// Listen briefly for "SCENARIO:<name>" (the laptop sends it repeatedly right
// after reset). The scenario has to be known before the flight controller
// starts, because some scenarios change settings.
String waitForScenario(unsigned long windowMs) {
  String line;
  unsigned long start = millis();
  while (millis() - start < windowMs) {
    while (Serial.available()) {
      char c = (char)Serial.read();
      if (c == '\n') {
        line.trim();
        if (line.startsWith("SCENARIO:")) return line.substring(9);
        line = "";
      } else if (c != '\r') {
        line += c;
      }
    }
    delay(10);
  }
  return "full";
}

void setup() {
  Serial.begin(115200);
  delay(2000);   // native USB takes a moment to reconnect after a reset

  String scenario = waitForScenario(1500);

  String errors;
  if (!fc::loadSettings(errors)) fc::halt("Can't load flight settings:\n" + errors);

  // The scenario's own settings: the sim's choices, made here in the app.
  FlightSettings& s = fc::settingsForEdit();
  s.safety.maxFlightTimeMs = 10UL * 60UL * 1000UL;   // room to fly the whole field in the sim
  if (scenario == "geofence") s.safety.geofenceRadiusM = 40.0f;      // trips soon after takeoff
  if (scenario == "timeout")  s.safety.maxFlightTimeMs = 18000UL;    // ~18 s -> time's-up landing
  if (scenario == "lowbatt")  sim.startCharge = 0.25f;               // pack only 25% charged
  sim.scenario = scenario;

  fc::begin(sim);

  if (scenario == "testroute") fc::setMission(makeTestRoute(settings().mission.route));
  logLine("[SCENARIO] selected: " + scenario);
}

void loop() {
  static String buf;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      if (buf.startsWith("DUMPLOG"))    sim.dumpLog();
      else if (buf.startsWith("PING:")) logLine("[SIM] Ready.");
      buf = "";
    } else if (c != '\r') {
      buf += c;
    }
  }
  delay(10);
}
