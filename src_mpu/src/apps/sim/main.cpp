// ============================================================================
// APP: sim -- the on-chip simulator. The REAL flight controller flies against
// a simulated world, all on the ESP, at the real 200 Hz rate.
//
//   flash:     pio run -e sim -t upload
//   settings:  pio run -e sim -t uploadfs        (the same data/ folder)
//   run:       ./simulate/run_hil.sh             (flashes, runs every scenario, checks them)
//
// The pieces:
//   flight/hardware/sim/   the fake parts (one file each) and SimWorld, what's
//                          TRUE: the QuadSim physics and the pack's charge.
//                          simHardware() in flight/hardware/HardwareIo.hpp
//                          plugs them in.
//   scenarios/             one file per test: what it does to the world, which
//                          plug it breaks, and what the pilot does
//   FlightLog.hpp          the flight recorder (/flight.csv, streamed back by DUMPLOG)
//
// Talks to the laptop over USB:
//   SCENARIO:<name>  pick the scenario (sent right after reset, before takeoff):
//                    full / geofence / gpsloss / timeout / rcloss / lowbatt /
//                    land / testroute / manual / stab   (default: full)
//   DUMPLOG          stream the recorded flight log back
//   PING:            liveness check
// ============================================================================

#include "flight/FlightController.hpp"
#include "flight/hardware/HardwareIo.hpp"   // simHardware(), SimWorld, SimRadio
#include "FlightLog.hpp"
#include "scenarios/FlyMission.hpp"
#include "scenarios/LowBattery.hpp"
#include "scenarios/GpsLoss.hpp"
#include "scenarios/RadioLoss.hpp"
#include "scenarios/LandSwitch.hpp"
#include "scenarios/ManualFlight.hpp"
#include "scenarios/Stabilization.hpp"

// --- every scenario ------------------------------------------------------------
std::vector<Scenario*> scenarios = {
  new FlyMission("full"),
  new FlyMission("geofence"),
  new FlyMission("timeout"),
  new FlyMission("testroute", {"first_mission"}),   // the real first-flight route
  new LowBattery(),
  new GpsLoss(),
  new RadioLoss(),
  new LandSwitch(),
  new ManualFlight(),
  new Stabilization(),
};

// --- the simulated world, and the fake transmitter the pilot holds -----------
SimWorld*  world     = new SimWorld();
SimRadio*  radio     = new SimRadio();
FlightLog* flightLog = new FlightLog(world);

SimRig    rig{world, radio, {}};
Scenario* scenario = nullptr;

// Listen briefly for "SCENARIO:<name>" (the laptop sends it repeatedly right
// after reset). The scenario has to be known before the settings load,
// because a scenario can bring its own override files.
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

  String name = waitForScenario(1500);
  for (Scenario* s : scenarios) if (name == s->name()) scenario = s;
  if (!scenario) fc::halt("Unknown scenario: " + name);

  // The settings, layered like appsettings files in .NET:
  //   flightsettings.json                  the drone
  //   + the scenario's own files           (testroute: the real first_mission.json)
  //   + flightsettings/sim.<scenario>.json this scenario's changes ({} if none).
  //                                          Every scenario has one; a missing
  //                                          file stops the sim.
  std::vector<String> overrides = scenario->settingsFiles();
  overrides.push_back("sim." + name);

  String errors;
  if (!fc::loadSettings(errors, overrides)) fc::halt("Can't load flight settings:\n" + errors);

  // The fake parts, where the real drone would use realHardware().
  rig.io = simHardware(world, radio);

  // The scenario may swap one for a broken version (gpsloss, rcloss).
  scenario->setup(rig);

  // "Build": start the world, then hand the flight controller its parts.
  world->begin();
  flightLog->begin();
  fc::begin(rig.io);
  logLine("[SCENARIO] selected: " + name);
}

void loop() {
  // 10 Hz: the scenario's pilot, then one row of the flight log.
  static unsigned long lastTickMs = 0;
  if (millis() - lastTickMs >= 100) {
    lastTickMs = millis();
    scenario->tick(rig);
    flightLog->sample();
  }

  // Commands from the laptop.
  static String buf;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      if (buf.startsWith("DUMPLOG"))    flightLog->dump();
      else if (buf.startsWith("PING:")) logLine("[SIM] Ready.");
      buf = "";
    } else if (c != '\r') {
      buf += c;
    }
  }
  delay(5);
}
