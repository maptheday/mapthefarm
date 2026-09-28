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

SimIo sim;

// Listen briefly for "SCENARIO:<name>" (the laptop sends it repeatedly right
// after reset). The scenario has to be known before the settings load,
// because a scenario can bring its own override file.
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

  // The settings, layered like appsettings files in .NET:
  //   flightsettings.json                  the drone
  //   + flightsettings/first_mission.json  (testroute only: the exact file the
  //                                          first_mission app flies)
  //   + flightsettings/sim.json            the sim's own changes (10-min limit)
  //   + flightsettings/sim.<scenario>.json if this scenario has one (e.g. geofence)
  std::vector<String> overrides;
  if (scenario == "testroute") overrides.push_back("first_mission");
  overrides.push_back("sim");
  if (fc::hasSettingsOverride("sim." + scenario)) overrides.push_back("sim." + scenario);

  String errors;
  if (!fc::loadSettings(errors, overrides)) fc::halt("Can't load flight settings:\n" + errors);

  // Not a flight setting: how charged the SIMULATED pack is (the drone itself
  // always assumes a full pack).
  if (scenario == "lowbatt") sim.startCharge = 0.25f;
  sim.scenario = scenario;

  fc::begin(sim);
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
