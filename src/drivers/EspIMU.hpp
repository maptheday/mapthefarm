#pragma once
#include "../interfaces/IIMU.hpp"

#ifdef ON_REAL_HARDWARE
#include <Arduino.h>
#include <Wire.h>
#include <cmath>

// ============================================================
//  EspIMU  —  Real MPU6050 driver for ESP32-S3
//
//  Communicates with the MPU6050 over I2C to read 6-axis
//  inertial data (3-axis accelerometer + 3-axis gyroscope).
//
//  I2C Wiring (ESP32-S3 default pins):
//    SDA → GPIO 8 (default I2C SDA)
//    SCL → GPIO 9 (default I2C SCL)
//    GND → GND
//    VCC → 3.3V
//    AD0 → GND (sets I2C address to 0x68; tie to VCC for 0x69)
//
//  The MPU6050 registers we use:
//    0x3B-0x40: Accelerometer X/Y/Z (int16)
//    0x43-0x48: Gyroscope X/Y/Z (int16)
//    0x1B:      Gyroscope Full-Scale Range (default ±250°/s)
//    0x1C:      Accelerometer Full-Scale Range (default ±2g)
//
//  Attitude estimation:
//    Roll  = atan2(accel_y, accel_z) in degrees  [right lean]
//    Pitch = atan2(-accel_x, sqrt(accel_y² + accel_z²)) in degrees [nose up]
//    Yaw   = integrated from gyro_z  [relative heading, drifts over time]
//
//  For a full AHRS, consider adding a magnetometer (HMC5883L)
//  to provide absolute yaw / true north heading.
// ============================================================

static constexpr uint8_t MPU6050_ADDR = 0x68;  // Default I2C address (AD0 tied to GND)

// MPU6050 register addresses
static constexpr uint8_t MPU6050_PWR_MGMT_1 = 0x6B;
static constexpr uint8_t MPU6050_GYRO_CONFIG = 0x1B;
static constexpr uint8_t MPU6050_ACCEL_CONFIG = 0x1C;
static constexpr uint8_t MPU6050_ACCEL_XOUT_H = 0x3B;
static constexpr uint8_t MPU6050_GYRO_XOUT_H = 0x43;

class EspIMU : public IIMU {
public:
    void initialize() override {
        // Initialize I2C if not already done
        Wire.begin(8, 9);  // SDA=GPIO8, SCL=GPIO9 (ESP32-S3 defaults)
        Wire.setClock(400000);  // 400 kHz I2C speed

        delay(100);  // Give MPU6050 time to power up

        // Wake up MPU6050 (clear sleep bit)
        writeRegister(MPU6050_PWR_MGMT_1, 0x00);
        delay(100);

        // Set gyroscope full-scale range: ±250°/s (most sensitive, good for quads)
        writeRegister(MPU6050_GYRO_CONFIG, 0x00);

        // Set accelerometer full-scale range: ±2g (most sensitive)
        writeRegister(MPU6050_ACCEL_CONFIG, 0x00);

        // Calibrate gyro at rest
        calibrateGyro();

        Serial.println("[IMU] EspIMU (MPU6050) initialised on I2C");
        Serial.print("[IMU] Gyro bias: X="); Serial.print(gyro_bias_x);
        Serial.print(" Y="); Serial.print(gyro_bias_y);
        Serial.print(" Z="); Serial.println(gyro_bias_z);
    }

    // Read roll angle (right lean in degrees)
    // From accelerometer: atan2(accel_y, accel_z)
    double readGyroRoll() override {
        // Note: sensor data is read once per cycle by Core 1 task
        // atan2 is in radians; convert to degrees
        double roll_rad = std::atan2(accel_y, accel_z);
        return roll_rad * 180.0 / M_PI;
    }

    // Read pitch angle (nose up in degrees)
    // From accelerometer: atan2(-accel_x, sqrt(accel_y² + accel_z²))
    double readGyroPitch() override {
        // Note: sensor data is read once per cycle by Core 1 task
        double denom = std::sqrt(accel_y * accel_y + accel_z * accel_z);
        double pitch_rad = std::atan2(-accel_x, denom);
        return pitch_rad * 180.0 / M_PI;
    }

    // Read yaw angle (heading in degrees, -180 to +180)
    // Integrated from gyro_z; will drift over time without mag correction
    double readYawDeg() override {
        readSensorData();

        // dt = time since last gyro read (in seconds)
        unsigned long now_ms = millis();
        static unsigned long last_ms = 0;
        double dt = (now_ms - last_ms) / 1000.0;
        last_ms = now_ms;

        if (dt > 0.01) dt = 0.01;  // Clamp dt to prevent large jumps
        if (dt <= 0.0) dt = 0.004; // Assume 250Hz if no time passed

        // Integrate gyro_z (subtract bias, convert to deg/s)
        double gyro_z_corrected = (gyro_z - gyro_bias_z) / 131.0;  // LSB sensitivity for ±250°/s
        yaw_integrated += gyro_z_corrected * dt;

        // Wrap yaw to -180 to +180 range
        while (yaw_integrated > 180.0) yaw_integrated -= 360.0;
        while (yaw_integrated < -180.0) yaw_integrated += 360.0;

        return yaw_integrated;
    }

private:
    // ── Sensor calibration ────────────────────────────────────
    double gyro_bias_x = 0.0, gyro_bias_y = 0.0, gyro_bias_z = 0.0;
    double yaw_integrated = 0.0;  // Accumulated yaw from gyro integration

    // ── Last-read sensor values (in raw units before conversion) ─
    double accel_x = 0.0, accel_y = 0.0, accel_z = 0.0;
    double gyro_x = 0.0, gyro_y = 0.0, gyro_z = 0.0;

    // Read all six axes from MPU6050 and convert to physical units
    void readSensorData() {
        // Read 14 bytes: accel_x/y/z (6 bytes) + temp (2 bytes) + gyro_x/y/z (6 bytes)
        uint8_t data[14];
        readRegisters(MPU6050_ACCEL_XOUT_H, data, 14);

        // Accelerometer: combine high and low bytes, then convert to g
        // LSB sensitivity: 16384 LSB/g for ±2g range
        int16_t ax_raw = (int16_t)((data[0] << 8) | data[1]);
        int16_t ay_raw = (int16_t)((data[2] << 8) | data[3]);
        int16_t az_raw = (int16_t)((data[4] << 8) | data[5]);

        accel_x = ax_raw / 16384.0;  // Convert to g
        accel_y = ay_raw / 16384.0;
        accel_z = az_raw / 16384.0;

        // Gyroscope: LSB sensitivity is 131 LSB/(°/s) for ±250°/s range
        // These are raw before converting to deg/s
        int16_t gx_raw = (int16_t)((data[8] << 8) | data[9]);
        int16_t gy_raw = (int16_t)((data[10] << 8) | data[11]);
        int16_t gz_raw = (int16_t)((data[12] << 8) | data[13]);

        gyro_x = gx_raw;
        gyro_y = gy_raw;
        gyro_z = gz_raw;
    }

    // Calibrate gyro by averaging 200 readings at rest
    void calibrateGyro() {
        Serial.println("[IMU] Calibrating gyro... keep drone still!");
        double sum_x = 0, sum_y = 0, sum_z = 0;
        const int samples = 200;

        for (int i = 0; i < samples; i++) {
            uint8_t data[6];
            readRegisters(MPU6050_GYRO_XOUT_H, data, 6);

            int16_t gx = (int16_t)((data[0] << 8) | data[1]);
            int16_t gy = (int16_t)((data[2] << 8) | data[3]);
            int16_t gz = (int16_t)((data[4] << 8) | data[5]);

            sum_x += gx;
            sum_y += gy;
            sum_z += gz;

            delay(10);
        }

        gyro_bias_x = sum_x / samples;
        gyro_bias_y = sum_y / samples;
        gyro_bias_z = sum_z / samples;

        Serial.println("[IMU] Gyro calibration complete!");
    }

    // Write one byte to a register
    void writeRegister(uint8_t reg, uint8_t value) {
        Wire.beginTransmission(MPU6050_ADDR);
        Wire.write(reg);
        Wire.write(value);
        Wire.endTransmission();
    }

    // Read 'count' bytes starting from 'reg'
    void readRegisters(uint8_t reg, uint8_t* data, int count) {
        Wire.beginTransmission(MPU6050_ADDR);
        Wire.write(reg);
        Wire.endTransmission();

        Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)count);
        for (int i = 0; i < count; i++) {
            if (Wire.available()) {
                data[i] = Wire.read();
            }
        }
    }
};

#endif // ON_REAL_HARDWARE
