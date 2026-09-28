#pragma once

// ============================================================================
// GPS service -- owns the TinyGPSPlus parser on Serial2 and turns the raw NMEA
// byte stream into a RawGpsReading. Parallel to the Compass service: one file,
// one sensor. Used by HardwareIo; the sim app supplies its own GPS instead.
// ============================================================================

#include <Arduino.h>
#include <TinyGPSPlus.h>
#include "../state/FlightSettings.hpp"   // settings().wiring.gpsRx, settings().wiring.gpsTx, settings().wiring.gpsBaud
#include "../models/SensorTypes.hpp"   // RawGpsReading

class Gps {
public:
  void begin() {
    Serial2.begin(settings().wiring.gpsBaud, SERIAL_8N1, settings().wiring.gpsRx, settings().wiring.gpsTx);
  }

  // Feed in any pending bytes. If a fresh, valid fix is available, fill `out`
  // and return true; otherwise return false (the fix is stale/absent).
  bool read(RawGpsReading& out) {
    while (Serial2.available() > 0) gps_.encode(Serial2.read());
    if (gps_.location.isValid() && gps_.location.age() < 2000) {
      out.lat       = gps_.location.lat();
      out.lon       = gps_.location.lng();
      out.fix       = true;
      out.sats      = gps_.satellites.value();
      out.speedMps  = gps_.speed.mps();
      out.lastFixMs = millis();
      return true;
    }
    return false;
  }

private:
  TinyGPSPlus gps_;
};

// The one Gps instance (defined in FlightController.hpp).
extern Gps gps;
