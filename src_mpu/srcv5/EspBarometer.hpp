#pragma once
#include "./IBarometer.hpp"

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BME280.h>

// ============================================================
//  EspBarometer  —  BME280 driver using Adafruit library
//
//  The Adafruit library handles all the raw register reads
//  and Bosch's compensation math internally — we just ask
//  for pressure and altitude directly.
//
//  I2C Wiring (ESP32-S3):
//    SDA → GPIO 16
//    SCL → GPIO 15
//    GND → GND
//    VCC → 3.3V
//    SDO → GND  (sets I2C address to 0x76)
//
//  Altitude is relative to ground level at startup (ground = 0.0m)
// ============================================================

class EspBarometer : public IBarometer {
public:
    void initialize() override {
        // Try 0x77 first (some boards), fall back to 0x76
        bool found = bme_.begin(0x77, &Wire);
        if (!found) found = bme_.begin(0x76, &Wire);

        if (!found) {
            Serial.println("[BARO] ERROR: BME280 not found. Check wiring.");
            Serial.println("[BARO] SDA→GPIO16, SCL→GPIO15, SDO→GND, VCC→3.3V");
            return;
        }

        // Normal mode: take readings continuously
        // Oversample x2 for pressure, temp, humidity
        // No IIR filter, 250ms between readings
        bme_.setSampling(
            Adafruit_BME280::MODE_NORMAL,
            Adafruit_BME280::SAMPLING_X2,  // temperature
            Adafruit_BME280::SAMPLING_X2,  // pressure
            Adafruit_BME280::SAMPLING_X2,  // humidity
            Adafruit_BME280::FILTER_OFF,
            Adafruit_BME280::STANDBY_MS_250
        );

        delay(50); // let first conversion complete

        // Average 20 readings to establish ground-level reference
        Serial.println("[BARO] Calibrating ground reference — keep still...");
        float sum = 0.0f;
        for (int i = 0; i < 20; i++) {
            sum += bme_.readPressure() / 100.0f; // Pa -> hPa
            delay(20);
        }
        groundLevelHpa_ = sum / 20.0f;

        Serial.print("[BARO] Ground reference: ");
        Serial.print(groundLevelHpa_, 2);
        Serial.println(" hPa");
        Serial.println("[BARO] BME280 ready.");
    }

    // Returns altitude in metres above the startup point (ground = 0.0)
    double readAltitudeMeters() override {
        // Adafruit handles the pressure → altitude formula internally
        // We pass our ground-level pressure as the reference
        return bme_.readAltitude(groundLevelHpa_);
    }

private:
    Adafruit_BME280 bme_;
    float groundLevelHpa_ = 1013.25f; // standard sea level — overwritten on init
};