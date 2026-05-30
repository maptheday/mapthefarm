#pragma once
#include "../interfaces/IBarometer.hpp"

#ifdef ON_REAL_HARDWARE
#include <Arduino.h>
#include <Wire.h>
#include <cmath>

// ============================================================
//  EspBarometer  —  Real BMP280 driver for ESP32-S3
//
//  Communicates with the BMP280 over I2C to read barometric
//  pressure and compute altitude via the barometric formula.
//
//  I2C Wiring (ESP32-S3 default pins):
//    SDA → GPIO 8 (default I2C SDA)
//    SCL → GPIO 9 (default I2C SCL)
//    GND → GND
//    VCC → 3.3V
//    SDO → GND (sets I2C address to 0x76; tie to VCC for 0x77)
//
//  BMP280 registers:
//    0xF7-0xF9: Pressure (3 bytes, 20-bit)
//    0xFA-0xFC: Temperature (3 bytes, 20-bit)
//    0x88-0xA1: Calibration coefficients (fixed at power-on)
//
//  Altitude computation:
//    Uses barometric formula: h = 44330 * (1 - (P/P0)^(1/5.255))
//    Where P0 is sea-level pressure (101325 Pa)
//    Requires temperature compensation for accuracy
//
//  Reference altitude:
//    On startup, reads pressure 10 times and averages → reference_altitude_m = 0
//    All future readings are relative to this baseline
// ============================================================

static constexpr uint8_t BMP280_ADDR = 0x76;  // Default I2C address (SDO tied to GND)

// BMP280 register addresses
static constexpr uint8_t BMP280_ID = 0xD8;
static constexpr uint8_t BMP280_RESET = 0xE0;
static constexpr uint8_t BMP280_STATUS = 0xF3;
static constexpr uint8_t BMP280_CTRL_MEAS = 0xF4;
static constexpr uint8_t BMP280_CONFIG = 0xF5;
static constexpr uint8_t BMP280_PRESS_MSB = 0xF7;
static constexpr uint8_t BMP280_TEMP_MSB = 0xFA;
static constexpr uint8_t BMP280_CALIB_START = 0x88;

class EspBarometer : public IBarometer {
public:
    void initialize() override {
        // Initialize I2C if not already done
        Wire.begin(8, 9);  // SDA=GPIO8, SCL=GPIO9 (ESP32-S3 defaults)
        Wire.setClock(400000);  // 400 kHz I2C speed

        delay(100);  // Give BMP280 time to power up

        // Verify chip ID (should be 0x58 for BMP280)
        uint8_t chip_id = readRegister(BMP280_ID);
        if (chip_id != 0x58) {
            Serial.print("[BARO] ERROR: Chip ID = 0x");
            Serial.println(chip_id, HEX);
            Serial.println("[BARO] Expected 0x58 — check I2C connection!");
        }

        // Read calibration coefficients from EEPROM
        readCalibration();

        // Configure BMP280:
        // CTRL_MEAS: 01 01 1011 = 0x5B
        //   - Temperature oversampling: 2x (01)
        //   - Pressure oversampling: 2x (01)
        //   - Mode: normal (11)
        writeRegister(BMP280_CTRL_MEAS, 0x5B);

        // CONFIG: 0000 0100 = 0x04
        //   - T standby: 125ms (000)
        //   - IIR filter: off (00)
        writeRegister(BMP280_CONFIG, 0x04);

        // Read baseline pressure (assume we're at ground level initially)
        // Average 10 readings to get a stable reference
        double pressure_sum = 0.0;
        for (int i = 0; i < 10; i++) {
            pressure_sum += readPressurePa();
            delay(10);
        }
        reference_pressure_pa = pressure_sum / 10.0;

        Serial.println("[BARO] EspBarometer (BMP280) initialised on I2C");
        Serial.print("[BARO] Reference pressure: ");
        Serial.print(reference_pressure_pa);
        Serial.println(" Pa");
    }

    // Returns altitude in meters above the reference (startup) point
    double readAltitudeMeters() override {
        double pressure_pa = readPressurePa();

        // Barometric formula: h = 44330 * (1 - (P/P0)^(1/5.255))
        // where P0 = reference pressure at ground
        double ratio = pressure_pa / reference_pressure_pa;
        if (ratio <= 0.0) ratio = 1.0;  // Safety check

        double altitude = 44330.0 * (1.0 - std::pow(ratio, 1.0 / 5.255));
        return altitude;
    }

private:
    // ── Reference pressure (set at startup) ──────────────────
    double reference_pressure_pa = 101325.0;

    // ── BMP280 calibration coefficients ──────────────────────
    int16_t  dig_T1, dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

    // ── Temperature compensation ─────────────────────────────
    int32_t t_fine = 0;

    // Read calibration coefficients from BMP280 EEPROM
    void readCalibration() {
        uint8_t calib[26];
        readRegisters(BMP280_CALIB_START, calib, 26);

        dig_T1 = (calib[1] << 8) | calib[0];
        dig_T2 = (int16_t)((calib[3] << 8) | calib[2]);
        dig_T3 = (int16_t)((calib[5] << 8) | calib[4]);

        dig_P1 = (calib[7] << 8) | calib[6];
        dig_P2 = (int16_t)((calib[9] << 8) | calib[8]);
        dig_P3 = (int16_t)((calib[11] << 8) | calib[10]);
        dig_P4 = (int16_t)((calib[13] << 8) | calib[12]);
        dig_P5 = (int16_t)((calib[15] << 8) | calib[14]);
        dig_P6 = (int16_t)((calib[17] << 8) | calib[16]);
        dig_P7 = (int16_t)((calib[19] << 8) | calib[18]);
        dig_P8 = (int16_t)((calib[21] << 8) | calib[20]);
        dig_P9 = (int16_t)((calib[23] << 8) | calib[22]);
    }

    // Read temperature from BMP280 and update t_fine (used for pressure compensation)
    int32_t readTemperature() {
        uint8_t data[3];
        readRegisters(BMP280_TEMP_MSB, data, 3);

        int32_t adc_T = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | (data[2] >> 4);

        // Temperature compensation (from BMP280 datasheet)
        int32_t var1 = (((adc_T >> 3) - ((int32_t)dig_T1 << 1)) * ((int32_t)dig_T2)) >> 11;
        int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
        t_fine = var1 + var2;

        return (t_fine * 5 + 128) >> 8;  // Temperature in 0.01°C units
    }

    // Read pressure from BMP280 (requires t_fine to be updated first)
    double readPressurePa() {
        // Update t_fine by reading temperature first
        readTemperature();

        uint8_t data[3];
        readRegisters(BMP280_PRESS_MSB, data, 3);

        int32_t adc_P = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | (data[2] >> 4);

        // Pressure compensation (from BMP280 datasheet)
        int64_t var1 = ((int64_t)t_fine) - 128000;
        int64_t var2 = var1 * var1 * (int64_t)dig_P6;
        var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
        var2 = var2 + (((int64_t)dig_P4) << 35);
        var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
        var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;

        if (var1 == 0) return 0;  // Avoid division by zero

        int64_t pressure = 1048576 - adc_P;
        pressure = (((pressure << 31) - var2) * 3125) / var1;
        var1 = (((int64_t)dig_P9) * (pressure >> 13) * (pressure >> 13)) >> 25;
        var2 = (((int64_t)dig_P8) * pressure) >> 19;
        pressure = ((pressure + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);

        return (double)pressure / 256.0;  // Return in Pa
    }

    // Write one byte to a register
    void writeRegister(uint8_t reg, uint8_t value) {
        Wire.beginTransmission(BMP280_ADDR);
        Wire.write(reg);
        Wire.write(value);
        Wire.endTransmission();
    }

    // Read one byte from a register
    uint8_t readRegister(uint8_t reg) {
        Wire.beginTransmission(BMP280_ADDR);
        Wire.write(reg);
        Wire.endTransmission();

        Wire.requestFrom((uint8_t)BMP280_ADDR, (uint8_t)1);
        if (Wire.available()) {
            return Wire.read();
        }
        return 0;
    }

    // Read multiple bytes starting from a register
    void readRegisters(uint8_t reg, uint8_t* data, int count) {
        Wire.beginTransmission(BMP280_ADDR);
        Wire.write(reg);
        Wire.endTransmission();

        Wire.requestFrom((uint8_t)BMP280_ADDR, (uint8_t)count);
        for (int i = 0; i < count; i++) {
            if (Wire.available()) {
                data[i] = Wire.read();
            }
        }
    }
};

#endif // ON_REAL_HARDWARE
