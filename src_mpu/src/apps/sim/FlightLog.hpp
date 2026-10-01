#pragma once

// ============================================================================
// FLIGHT LOG -- the sim's flight recorder. Ten times a second it writes one CSV
// row to /flight.csv on the drone's flash: the TRUE position and tilt (from the
// SimWorld), plus what the flight controller was doing. DUMPLOG streams it back
// to the laptop, where simulate/scenarios_esp checks it and draws the map.
// ============================================================================

#include <Arduino.h>
#include <LittleFS.h>
#include "flight/FlightController.hpp"   // fc::phase(), fc::batteryState(), ...
#include "SimWorld.hpp"

class FlightLog {
public:
  FlightLog(SimWorld* world) : world_(world) {}

  void begin() {
    t0Ms_ = millis();
    file_ = LittleFS.open(PATH, "w");
    if (!file_) return;
    file_.printf("# home_lat=%.6f home_lon=%.6f\n", world_->homeLat(), world_->homeLon());
    file_.println("t_s,phase,north_m,east_m,up_ft,roll_deg,pitch_deg,dist_m,wp,cell_v,mah_est");
    file_.flush();
  }

  // One row (call at 10 Hz).
  void sample() {
    if (!file_) return;
    Truth       t     = world_->truth();
    FlightPhase phase = fc::phase();
    RawBattery  b     = fc::batteryState();
    float dist = sqrtf(t.northM * t.northM + t.eastM * t.eastM);
    file_.printf("%.2f,%s,%.1f,%.1f,%.1f,%.2f,%.2f,%.1f,%d,%.3f,%.0f\n",
                 (millis() - t0Ms_) / 1000.0f, phaseName(phase), t.northM, t.eastM, t.upFt,
                 t.rollDeg, t.pitchDeg, dist, fc::missionWaypointIndex(), b.cellVolts, b.mAhUsed);
    if (phase == PHASE_LANDED) file_.flush();
  }

  // DUMPLOG: stream the recorded log back over USB between markers.
  void dump() {
    if (file_) file_.flush();
    File f = LittleFS.open(PATH, "r");
    if (!f) { logLine("[SIM] no log to dump"); return; }
    Serial.println("===ONBOARD_LOG_START===");
    while (f.available()) Serial.write(f.read());
    Serial.println("===ONBOARD_LOG_END===");
    f.close();
  }

private:
  static constexpr const char* PATH = "/flight.csv";
  SimWorld*     world_;
  File          file_;
  unsigned long t0Ms_ = 0;
};
