#pragma once
#include "../interfaces/IBarometer.hpp"

#ifdef ON_REAL_HARDWARE
#include <Arduino.h>
#include <Wire.h>
#include <cmath>

// ============================================================
//  EspBarometer  —  Real BME280 driver for ESP32-S3
//
//  The BME280 is an upgraded version of the BMP280. It adds
//  humidity sensing but the pressure and temperature registers
//  work identically. The key difference is the chip ID (0x60
//  instead of 0x58) and slightly different calibration data.
//
//  I2C Wiring (ESP32-S3):
//    SDA → GPIO 8
//    SCL → GPIO 9
//    GND → GND
//    VCC → 3.3V
//    SDO → GND  (sets I2C address to 0x76)
//
//  BME280 registers used:
//    0xD0: chip ID (should read 0x60)
//    0xF3: status
//    0xF4: ctrl_meas  (temp + pressure oversampling + mode)
//    0xF5: config     (standby time + IIR filter)
//    0xF7-0xF9: pressure raw (20-bit)
//    0xFA-0xFC: temperature raw (20-bit)
//    0x88-0x9F: temperature + pressure calibration (26 bytes)
//    0xE1-0xE7: humidity calibration (not used here)
//
//  Altitude formula:
//    h = 44330 * (1 - (P / P_ref) ^ (1/5.255))
//    where P_ref = pressure at ground level on startup
// ============================================================

static constexpr uint8_t BME280_ADDR         = 0x76;
static constexpr uint8_t BME280_REG_ID       = 0xD0;
static constexpr uint8_t BME280_REG_RESET    = 0xE0;
static constexpr uint8_t BME280_REG_CTRL     = 0xF4;
static constexpr uint8_t BME280_REG_CONFIG   = 0xF5;
static constexpr uint8_t BME280_REG_PRESS    = 0xF7;
static constexpr uint8_t BME280_REG_TEMP     = 0xFA;
static constexpr uint8_t BME280_REG_CALIB    = 0x88;
static constexpr uint8_t BME280_CHIP_ID      = 0x60;  // BME280 returns 0x60, BMP280 returns 0x58

class EspBarometer : public IBarometer {
public:
    void initialize() override {
        Wire.begin(15, 16);
        Wire.setClock(400000);
        delay(100);

        // Verify chip — catches wrong wiring or wrong I2C address early
        uint8_t id = readRegister(BME280_REG_ID);
        if (id != BME280_CHIP_ID) {
            Serial.print("[BARO] ERROR: Expected chip ID 0x60 (BME280), got 0x");
            Serial.println(id, HEX);
            Serial.println("[BARO] Check: SDA→GPIO8, SCL→GPIO9, SDO→GND, VCC→3.3V");
            return;
        }

        // Soft reset to clear any leftover state
        writeRegister(BME280_REG_RESET, 0xB6);
        delay(10);

        // Read calibration coefficients baked into the chip at the factory
        readCalibration();

        // ctrl_meas: 0x57 = temp oversample x2 (010), pressure oversample x2 (010), normal mode (11)
        writeRegister(BME280_REG_CTRL, 0x57);

        // config: 0x00 = 0.5ms standby, no IIR filter
        writeRegister(BME280_REG_CONFIG, 0x00);

        delay(100);

        // Average 20 readings at startup to establish ground-level reference pressure
        Serial.println("[BARO] Calibrating ground reference — keep still...");
        double sum = 0.0;
        for (int i = 0; i < 20; i++) {
            sum += readPressurePa();
            delay(20);
        }
        reference_pressure_pa = sum / 20.0;

        Serial.println("[BARO] EspBarometer (BME280) initialised");
        Serial.print("[BARO] Ground reference: ");
        Serial.print(reference_pressure_pa, 1);
        Serial.println(" Pa");
    }

    // Returns altitude in metres above the startup point (ground = 0.0)
    double readAltitudeMeters() override {
        double pressure = readPressurePa();
        if (pressure <= 0.0) return 0.0;

        double ratio = pressure / reference_pressure_pa;
        return 44330.0 * (1.0 - std::pow(ratio, 1.0 / 5.255));
    }

private:
    double reference_pressure_pa = 101325.0;

    // ── BME280 calibration coefficients ──────────────────────
    // These are unique to every chip — read once on startup
    uint16_t dig_T1;
    int16_t  dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    int32_t  t_fine = 0;  // shared between temp and pressure compensation

    void readCalibration() {
        uint8_t c[26];
        readRegisters(BME280_REG_CALIB, c, 26);

        dig_T1 = (uint16_t)((c[1] << 8) | c[0]);
        dig_T2 = (int16_t) ((c[3] << 8) | c[2]);
        dig_T3 = (int16_t) ((c[5] << 8) | c[4]);

        dig_P1 = (uint16_t)((c[7]  << 8) | c[6]);
        dig_P2 = (int16_t) ((c[9]  << 8) | c[8]);
        dig_P3 = (int16_t) ((c[11] << 8) | c[10]);
        dig_P4 = (int16_t) ((c[13] << 8) | c[12]);
        dig_P5 = (int16_t) ((c[15] << 8) | c[14]);
        dig_P6 = (int16_t) ((c[17] << 8) | c[16]);
        dig_P7 = (int16_t) ((c[19] << 8) | c[18]);
        dig_P8 = (int16_t) ((c[21] << 8) | c[20]);
        dig_P9 = (int16_t) ((c[23] << 8) | c[22]);
    }

    // Must call readTemperatureRaw() first to update t_fine
    // which pressure compensation depends on
    void readTemperatureRaw() {
        uint8_t d[3];
        readRegisters(BME280_REG_TEMP, d, 3);
        int32_t adc = ((int32_t)d[0] << 12) | ((int32_t)d[1] << 4) | (d[2] >> 4);

        // BME280 datasheet compensation formula (verbatim)
        int32_t v1 = (((adc >> 3) - ((int32_t)dig_T1 << 1)) * (int32_t)dig_T2) >> 11;
        int32_t v2 = (((((adc >> 4) - (int32_t)dig_T1) *
                        ((adc >> 4) - (int32_t)dig_T1)) >> 12) *
                       (int32_t)dig_T3) >> 14;
        t_fine = v1 + v2;
    }

    double readPressurePa() {
        readTemperatureRaw();  // updates t_fine needed for pressure compensation

        uint8_t d[3];
        readRegisters(BME280_REG_PRESS, d, 3);
        int32_t adc = ((int32_t)d[0] << 12) | ((int32_t)d[1] << 4) | (d[2] >> 4);

        // BME280 datasheet compensation formula (verbatim, 64-bit)
        int64_t v1 = (int64_t)t_fine - 128000;
        int64_t v2 = v1 * v1 * (int64_t)dig_P6;
        v2 += (v1 * (int64_t)dig_P5) << 17;
        v2 += ((int64_t)dig_P4) << 35;
        v1  = ((v1 * v1 * (int64_t)dig_P3) >> 8) + ((v1 * (int64_t)dig_P2) << 12);
        v1  = ((((int64_t)1 << 47) + v1) * (int64_t)dig_P1) >> 33;
        if (v1 == 0) return 0.0;

        int64_t p = 1048576 - adc;
        p = (((p << 31) - v2) * 3125) / v1;
        v1 = ((int64_t)dig_P9 * (p >> 13) * (p >> 13)) >> 25;
        v2 = ((int64_t)dig_P8 * p) >> 19;
        p  = ((p + v1 + v2) >> 8) + ((int64_t)dig_P7 << 4);

        return (double)p / 256.0;
    }

    void writeRegister(uint8_t reg, uint8_t val) {
        Wire.beginTransmission(BME280_ADDR);
        Wire.write(reg);
        Wire.write(val);
        Wire.endTransmission();
    }

    uint8_t readRegister(uint8_t reg) {
        Wire.beginTransmission(BME280_ADDR);
        Wire.write(reg);
        Wire.endTransmission();
        Wire.requestFrom((uint8_t)BME280_ADDR, (uint8_t)1);
        return Wire.available() ? Wire.read() : 0;
    }

    void readRegisters(uint8_t reg, uint8_t* buf, int count) {
        Wire.beginTransmission(BME280_ADDR);
        Wire.write(reg);
        Wire.endTransmission();
        Wire.requestFrom((uint8_t)BME280_ADDR, (uint8_t)count);
        for (int i = 0; i < count; i++)
            buf[i] = Wire.available() ? Wire.read() : 0;
    }
};

#endif // ON_REAL_HARDWARE