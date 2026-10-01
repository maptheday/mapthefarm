#pragma once

// ============================================================================
// FLIGHT SETTINGS -- everything a person might want to tune, loaded from
// /flightsettings.json on the drone's flash (like appsettings.json in C#).
//
// The flight controller has NO built-in values for these. If the file is
// missing, or any setting is missing or the wrong type, loadFlightSettings()
// fails and lists every problem, and the app refuses to fly. A drone that
// silently falls back to "some default" is how surprises happen.
//
// The file lives in the project's data/ folder. Put it on the drone with:
//     pio run -e fly -t uploadfs
//
// OVERRIDE FILES (like appsettings.Development.json): an app can ask for extra
// files layered on top, e.g. the first_mission app loads
//     /flightsettings.json  then  /flightsettings/first_mission.json
// Each override file holds ONLY what it changes. Sections merge key by key;
// a list (like mission.route) is replaced whole. Unlike .NET, an override an
// app asks for is REQUIRED: if it's missing, the app refuses to run, rather
// than quietly flying the base settings (e.g. the full route instead of the
// short test route).
//
// What is NOT here: true internals that nobody should tune in the field (loop
// rates, radio protocol constants, filter internals). Those stay in code, in
// FlightConstants.hpp and next to the code that uses them.
// ============================================================================

#include <Arduino.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <vector>
#include "FlightConstants.hpp"   // loop rates and other true internals

// One point of a mission route.
struct Waypoint {
  double lat;
  double lon;
  float  altFt;
};

struct PidGains {
  float kp, ki, kd;   // proportional, integral, derivative
  float min, max;     // output limits
};

struct FlightSettings {
  // The airframe: how this particular drone flies.
  struct {
    float    hoverThrottle;    // feed-forward: roughly the throttle it takes to hover
    PidGains altitudePid;      // feet of error -> throttle trim
    PidGains rollPid;          // degrees of error -> motor difference
    PidGains pitchPid;
    PidGains yawPid;
    float    yawRateDamping;   // extra push against spin, per deg/s
    PidGains navPid;           // metres of error -> target lean (degrees)
  } airframe;

  // How the automatic flight behaves.
  struct {
    float         takeoffAltitudeFt;
    float         climbRateFps;         // takeoff climb speed
    float         landDescentRateFps;
    float         waypointAcceptRadiusM;
    float         waypointLookaheadM;   // line-following "carrot" distance
    unsigned long missionCompleteHoverMs;
  } flight;

  // The safety net (Failsafes.hpp).
  struct {
    unsigned long maxFlightTimeMs;      // time's up -> land where it is
    float         geofenceRadiusM;      // too far from launch -> come home
    unsigned long gpsLossAbortMs;       // GPS gone this long -> land
    unsigned long radioLossTimeoutMs;   // radio silent this long -> come home / land
    float         radioLossLandBelowFt; // ...but land in place if lower than this
    float         rtlAltitudeFt;
    unsigned long rtlSettleMs;
  } safety;

  // The flight pack and the fuel gauge (Battery.hpp).
  struct {
    int   cells;
    float capacityMah;
    float warningUsed;       // fraction used (estimated) -> land
    float criticalUsed;
    float cellWarningV;      // per cell, filtered, under load -> land
    float cellCriticalV;
    float cellTakeoffMinV;   // per cell, at rest -> START refused below this
    float dividerScale;      // voltage divider ratio (tune with a multimeter)
    float currentIdleA;      // virtual current sensor: electronics only
    float currentMotorFullA; // virtual current sensor: one motor at full throttle
  } battery;

  // Flying by hand with the sticks (ManualPhase.hpp).
  struct {
    float maxLeanDeg;
    float climbRateFps;
    float yawRateDps;
    float stickDeadband;
    bool  positionHold;
  } manual;

  // Which radio channel does what, numbered like the radio's own screen
  // (CH1..CH16).
  struct {
    int roll, pitch, throttle, yaw;
    int start, stop, manual, land;
    int switchUpAbove;   // microseconds: a switch reads ~1000 down, ~1500 middle, ~2000 up
  } radio;

  // Where everything is plugged in.
  struct {
    int motorPins[4];      // M1 front-left, M2 front-right, M3 rear-left, M4 rear-right
    int escPwmHz;
    int i2cSda, i2cScl;
    int gpsRx, gpsTx, gpsBaud;
    int radioRx;
    int batteryAdc;
  } wiring;

  struct {
    unsigned long compassDurationMs;   // how long to rotate the drone for
  } calibration;

  // The route MISSION flies. An app can replace it (fc::setMission).
  struct {
    String                name;
    std::vector<Waypoint> route;
  } mission;
};

// The settings the running flight controller uses. Set once by fc::begin().
inline FlightSettings& flightSettingsStorage() { static FlightSettings s; return s; }
inline const FlightSettings& settings() { return flightSettingsStorage(); }

// ---------------------------------------------------------------------------
// Loading. Every read goes through one of these helpers, which record a clear
// message ("battery.capacityMah is missing") instead of quietly using zero.
// ---------------------------------------------------------------------------
namespace settings_json {

inline void need(JsonVariantConst v, const char* path, float& out, String& errors) {
  if (v.is<float>() || v.is<int>()) out = v.as<float>();
  else errors += String("  - ") + path + (v.isNull() ? " is missing\n" : " must be a number\n");
}
inline void need(JsonVariantConst v, const char* path, int& out, String& errors) {
  if (v.is<int>()) out = v.as<int>();
  else errors += String("  - ") + path + (v.isNull() ? " is missing\n" : " must be a whole number\n");
}
inline void need(JsonVariantConst v, const char* path, unsigned long& out, String& errors) {
  if (v.is<int>() && v.as<long>() >= 0) out = v.as<unsigned long>();
  else errors += String("  - ") + path + (v.isNull() ? " is missing\n" : " must be a whole number\n");
}
inline void need(JsonVariantConst v, const char* path, bool& out, String& errors) {
  if (v.is<bool>()) out = v.as<bool>();
  else errors += String("  - ") + path + (v.isNull() ? " is missing\n" : " must be true or false\n");
}
inline void needPid(JsonVariantConst v, const char* path, PidGains& out, String& errors) {
  String p(path);
  need(v["kp"],  (p + ".kp").c_str(),  out.kp,  errors);
  need(v["ki"],  (p + ".ki").c_str(),  out.ki,  errors);
  need(v["kd"],  (p + ".kd").c_str(),  out.kd,  errors);
  need(v["min"], (p + ".min").c_str(), out.min, errors);
  need(v["max"], (p + ".max").c_str(), out.max, errors);
}

}  // namespace settings_json

namespace settings_json {

// Read one JSON file from the drone's flash into `doc`.
inline bool readJsonFile(const String& path, JsonDocument& doc, String& errors) {
  File f = LittleFS.open(path.c_str(), "r");
  if (!f) {
    errors += "  - " + path + " not found. Upload the data/ folder with:\n"
              "      pio run -e fly -t uploadfs\n";
    return false;
  }
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) {
    errors += "  - " + path + " isn't valid JSON: " + err.c_str() + "\n";
    return false;
  }
  return true;
}

// Lay `over` on top of `base`: objects merge key by key (recursively);
// anything else -- a number, a string, a whole list -- replaces what was there.
inline void mergeJson(JsonVariant base, JsonVariantConst over) {
  if (!over.is<JsonObjectConst>() || !base.is<JsonObject>()) {
    base.set(over);
    return;
  }
  for (JsonPairConst kv : over.as<JsonObjectConst>()) {
    JsonVariant slot = base[kv.key()];
    if (kv.value().is<JsonObjectConst>() && slot.is<JsonObject>()) mergeJson(slot, kv.value());
    else base[kv.key()] = kv.value();
  }
}

}  // namespace settings_json

// The flash path of a named override file: "first_mission" ->
// "/flightsettings/first_mission.json". (A folder, not "flightsettings.
// first_mission.json": the flash file system allows only ~32 characters per
// file name.)
inline String settingsOverridePath(const String& name) {
  return "/flightsettings/" + name + ".json";
}

// Load /flightsettings.json, then each override file in `overrides` (in order),
// into `out`. Returns false (with every problem listed in `errors`) if a file
// is missing or broken, or if any setting is missing or the wrong type once
// everything is merged.
inline bool loadFlightSettings(FlightSettings& out, String& errors,
                               const std::vector<String>& overrides = {}) {
  using namespace settings_json;
  errors = "";

  if (!LittleFS.begin(false)) {
    errors = "  - the drone's file system isn't set up. Upload the settings with:\n"
             "      pio run -e fly -t uploadfs\n";
    return false;
  }
  JsonDocument doc;
  if (!readJsonFile("/flightsettings.json", doc, errors)) return false;
  for (const String& name : overrides) {
    JsonDocument over;
    if (!readJsonFile(settingsOverridePath(name), over, errors)) return false;
    mergeJson(doc.as<JsonVariant>(), over.as<JsonVariantConst>());
  }

  JsonVariantConst a = doc["airframe"];
  need(a["hoverThrottle"],  "airframe.hoverThrottle",  out.airframe.hoverThrottle,  errors);
  needPid(a["altitudePid"], "airframe.altitudePid",    out.airframe.altitudePid,    errors);
  needPid(a["rollPid"],     "airframe.rollPid",        out.airframe.rollPid,        errors);
  needPid(a["pitchPid"],    "airframe.pitchPid",       out.airframe.pitchPid,       errors);
  needPid(a["yawPid"],      "airframe.yawPid",         out.airframe.yawPid,         errors);
  need(a["yawRateDamping"], "airframe.yawRateDamping", out.airframe.yawRateDamping, errors);
  needPid(a["navPid"],      "airframe.navPid",         out.airframe.navPid,         errors);

  JsonVariantConst fl = doc["flight"];
  need(fl["takeoffAltitudeFt"],      "flight.takeoffAltitudeFt",      out.flight.takeoffAltitudeFt,      errors);
  need(fl["climbRateFps"],           "flight.climbRateFps",           out.flight.climbRateFps,           errors);
  need(fl["landDescentRateFps"],     "flight.landDescentRateFps",     out.flight.landDescentRateFps,     errors);
  need(fl["waypointAcceptRadiusM"],  "flight.waypointAcceptRadiusM",  out.flight.waypointAcceptRadiusM,  errors);
  need(fl["waypointLookaheadM"],     "flight.waypointLookaheadM",     out.flight.waypointLookaheadM,     errors);
  need(fl["missionCompleteHoverMs"], "flight.missionCompleteHoverMs", out.flight.missionCompleteHoverMs, errors);

  JsonVariantConst sf = doc["safety"];
  need(sf["maxFlightTimeMs"],      "safety.maxFlightTimeMs",      out.safety.maxFlightTimeMs,      errors);
  need(sf["geofenceRadiusM"],      "safety.geofenceRadiusM",      out.safety.geofenceRadiusM,      errors);
  need(sf["gpsLossAbortMs"],       "safety.gpsLossAbortMs",       out.safety.gpsLossAbortMs,       errors);
  need(sf["radioLossTimeoutMs"],   "safety.radioLossTimeoutMs",   out.safety.radioLossTimeoutMs,   errors);
  need(sf["radioLossLandBelowFt"], "safety.radioLossLandBelowFt", out.safety.radioLossLandBelowFt, errors);
  need(sf["rtlAltitudeFt"],        "safety.rtlAltitudeFt",        out.safety.rtlAltitudeFt,        errors);
  need(sf["rtlSettleMs"],          "safety.rtlSettleMs",          out.safety.rtlSettleMs,          errors);

  JsonVariantConst b = doc["battery"];
  need(b["cells"],             "battery.cells",             out.battery.cells,             errors);
  need(b["capacityMah"],       "battery.capacityMah",       out.battery.capacityMah,       errors);
  need(b["warningUsed"],       "battery.warningUsed",       out.battery.warningUsed,       errors);
  need(b["criticalUsed"],      "battery.criticalUsed",      out.battery.criticalUsed,      errors);
  need(b["cellWarningV"],      "battery.cellWarningV",      out.battery.cellWarningV,      errors);
  need(b["cellCriticalV"],     "battery.cellCriticalV",     out.battery.cellCriticalV,     errors);
  need(b["cellTakeoffMinV"],   "battery.cellTakeoffMinV",   out.battery.cellTakeoffMinV,   errors);
  need(b["dividerScale"],      "battery.dividerScale",      out.battery.dividerScale,      errors);
  need(b["currentIdleA"],      "battery.currentIdleA",      out.battery.currentIdleA,      errors);
  need(b["currentMotorFullA"], "battery.currentMotorFullA", out.battery.currentMotorFullA, errors);

  JsonVariantConst m = doc["manual"];
  need(m["maxLeanDeg"],    "manual.maxLeanDeg",    out.manual.maxLeanDeg,    errors);
  need(m["climbRateFps"],  "manual.climbRateFps",  out.manual.climbRateFps,  errors);
  need(m["yawRateDps"],    "manual.yawRateDps",    out.manual.yawRateDps,    errors);
  need(m["stickDeadband"], "manual.stickDeadband", out.manual.stickDeadband, errors);
  need(m["positionHold"],  "manual.positionHold",  out.manual.positionHold,  errors);

  JsonVariantConst r = doc["radio"];
  JsonVariantConst ch = r["channels"];
  need(ch["roll"],     "radio.channels.roll",     out.radio.roll,     errors);
  need(ch["pitch"],    "radio.channels.pitch",    out.radio.pitch,    errors);
  need(ch["throttle"], "radio.channels.throttle", out.radio.throttle, errors);
  need(ch["yaw"],      "radio.channels.yaw",      out.radio.yaw,      errors);
  need(ch["start"],    "radio.channels.start",    out.radio.start,    errors);
  need(ch["stop"],     "radio.channels.stop",     out.radio.stop,     errors);
  need(ch["manual"],   "radio.channels.manual",   out.radio.manual,   errors);
  need(ch["land"],     "radio.channels.land",     out.radio.land,     errors);
  need(r["switchUpAbove"], "radio.switchUpAbove", out.radio.switchUpAbove, errors);

  JsonVariantConst w = doc["wiring"];
  JsonArrayConst motorPins = w["motorPins"];
  if (motorPins.size() == 4) {
    for (int i = 0; i < 4; i++) need(motorPins[i], "wiring.motorPins[]", out.wiring.motorPins[i], errors);
  } else {
    errors += "  - wiring.motorPins must list exactly 4 pins (M1..M4)\n";
  }
  need(w["escPwmHz"],   "wiring.escPwmHz",   out.wiring.escPwmHz,   errors);
  need(w["i2cSda"],     "wiring.i2cSda",     out.wiring.i2cSda,     errors);
  need(w["i2cScl"],     "wiring.i2cScl",     out.wiring.i2cScl,     errors);
  need(w["gpsRx"],      "wiring.gpsRx",      out.wiring.gpsRx,      errors);
  need(w["gpsTx"],      "wiring.gpsTx",      out.wiring.gpsTx,      errors);
  need(w["gpsBaud"],    "wiring.gpsBaud",    out.wiring.gpsBaud,    errors);
  need(w["radioRx"],    "wiring.radioRx",    out.wiring.radioRx,    errors);
  need(w["batteryAdc"], "wiring.batteryAdc", out.wiring.batteryAdc, errors);

  need(doc["calibration"]["compassDurationMs"], "calibration.compassDurationMs",
       out.calibration.compassDurationMs, errors);

  JsonVariantConst mi = doc["mission"];
  out.mission.name = mi["name"] | "";
  out.mission.route.clear();
  JsonArrayConst route = mi["route"];
  if (route.isNull() || route.size() == 0) {
    errors += "  - mission.route needs at least one waypoint\n";
  } else {
    for (JsonVariantConst p : route) {
      Waypoint wp{};
      float lat = 0, lon = 0;
      need(p["lat"],   "mission.route[].lat",   lat,      errors);
      need(p["lon"],   "mission.route[].lon",   lon,      errors);
      need(p["altFt"], "mission.route[].altFt", wp.altFt, errors);
      wp.lat = p["lat"].as<double>();   // read again at full precision
      wp.lon = p["lon"].as<double>();
      out.mission.route.push_back(wp);
    }
  }

  return errors.length() == 0;
}
