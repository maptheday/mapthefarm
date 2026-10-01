#pragma once

// ============================================================================
// BN-880 GPS -- the real drone's IGps. Owns the TinyGPSPlus parser on Serial2
// and turns the raw NMEA byte stream into a RawGpsReading.
// ============================================================================

#include <Arduino.h>
#include <TinyGPSPlus.h>
#include "../../FlightIo.hpp"               // IGps
#include "../../state/FlightSettings.hpp"   // settings().wiring.gpsRx / gpsTx / gpsBaud

class Bn880Gps : public IGps {
public:
  void begin() override {
    Serial2.begin(settings().wiring.gpsBaud, SERIAL_8N1, settings().wiring.gpsRx, settings().wiring.gpsTx);
  }

  // Feed in any pending bytes. A fresh, valid fix comes back with fix = true;
  // a stale or absent one with fix = false.
  RawGpsReading read() override {
    while (Serial2.available() > 0) gps_.encode(Serial2.read());
    RawGpsReading out;
    if (gps_.location.isValid() && gps_.location.age() < 2000) {
      out.lat       = gps_.location.lat();
      out.lon       = gps_.location.lng();
      out.fix       = true;
      out.sats      = gps_.satellites.value();
      out.speedMps  = gps_.speed.mps();
      out.lastFixMs = millis();
    }
    return out;
  }

private:
  TinyGPSPlus gps_;
};
