#pragma once

// ============================================================================
// MPU6050 IMU -- the real drone's IImu. Owns the MPU6050 (accelerometer +
// gyro) and the Madgwick filter that fuses them into roll/pitch/yaw.
// ============================================================================

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MadgwickAHRS.h>
#include "../../FlightIo.hpp"              // IImu
#include "../../state/FlightConstants.hpp" // PHYSICS_LOOP_HZ
#include "../../services/Log.hpp"          // logLine
#include "I2cBus.hpp"                   // startI2c

class Mpu6050Imu : public IImu {
public:
  void begin() override {
    startI2c();
    logLine("[IMU] Initializing MPU6050...");
    if (!mpu_.begin()) {
      logLine("[IMU] ERROR: MPU6050 not found.");
      while (1) { delay(10); }
    }
    logLine("[IMU] MPU6050 ready.");
    filter_.begin(PHYSICS_LOOP_HZ);
  }

  // Read the sensor, advance the fusion filter over `dt` seconds, and return
  // the fused roll/pitch/yaw (stored in gyroX/Y/Z) plus raw accel + temperature.
  RawImuReading read(float dt) override {
    sensors_event_t a;
    sensors_event_t g;
    sensors_event_t temp;
    mpu_.getEvent(&a, &g, &temp);

    float gx = g.gyro.x * 57.2958f;
    float gy = g.gyro.y * 57.2958f;
    float gz = g.gyro.z * 57.2958f;

    if (dt > 0 && dt < 1.0f) {
      filter_.updateIMU(gx, gy, gz, a.acceleration.x, a.acceleration.y, a.acceleration.z);
    }

    RawImuReading out;
    out.gyroX = filter_.getRoll();
    out.gyroY = filter_.getPitch();
    out.gyroZ = filter_.getYaw();
    // Raw yaw rate, before the filter. The MPU6050's z axis points up when the
    // board lies flat, so a positive gz is a counter-clockwise turn -- the
    // opposite of compass heading, hence the minus. If the board is mounted
    // upside down, drop the minus. (Bench check: turn the drone clockwise by
    // hand; yawRateDps must read positive.)
    out.yawRateDps = -gz;
    out.accX  = a.acceleration.x;
    out.accY  = a.acceleration.y;
    out.accZ  = a.acceleration.z;
    out.temp  = temp.temperature;
    return out;
  }

private:
  Adafruit_MPU6050 mpu_;
  Madgwick         filter_;
};
