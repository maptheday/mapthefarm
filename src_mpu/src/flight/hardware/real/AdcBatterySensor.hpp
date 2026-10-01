#pragma once

// ============================================================================
// ADC BATTERY SENSOR -- the real drone's IBatterySensor. The pack voltage,
// read on an ESP32 pin through a two-resistor divider (the pin can only take
// ~3 V, so the divider scales the pack down; dividerScale scales it back up).
// Reads 0 if nothing is wired.
// ============================================================================

#include <Arduino.h>
#include "../../FlightIo.hpp"               // IBatterySensor
#include "../../state/FlightSettings.hpp"   // settings().wiring.batteryAdc, settings().battery.dividerScale

class AdcBatterySensor : public IBatterySensor {
public:
  void begin() override {
    analogSetPinAttenuation(settings().wiring.batteryAdc, ADC_11db);   // measure up to ~3 V on the pin
  }

  float readPackVolts() override {
    return analogReadMilliVolts(settings().wiring.batteryAdc) / 1000.0f *
           settings().battery.dividerScale;
  }
};
