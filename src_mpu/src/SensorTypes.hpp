#pragma once

// Plain data objects representing the latest sensor measurements. The
// controller uses these types without knowing which sensor produced them.

struct RawImuReading {
  float accX  = 0.0f;
  float accY  = 0.0f;
  float accZ  = 0.0f;
  // These fields hold Madgwick roll/pitch/yaw angles in this project.
  float gyroX = 0.0f;
  float gyroY = 0.0f;
  float gyroZ = 0.0f;
  float temp  = 0.0f;

  RawImuReading& operator=(const volatile RawImuReading& other) {
    accX = other.accX; accY = other.accY; accZ = other.accZ;
    gyroX = other.gyroX; gyroY = other.gyroY;
    gyroZ = other.gyroZ; temp = other.temp;
    return *this;
  }
};

struct RawGpsReading {
  double lat = 0.0;
  double lon = 0.0;
  bool   fix = false;
  int    sats = 0;
  float  speedMps = 0.0f;
  unsigned long lastFixMs = 0;

  RawGpsReading& operator=(const volatile RawGpsReading& other) {
    lat = other.lat; lon = other.lon; fix = other.fix;
    sats = other.sats; speedMps = other.speedMps;
    lastFixMs = other.lastFixMs;
    return *this;
  }
};

struct RawSensors {
  RawImuReading imu;
  RawGpsReading gps;
  float compassHeadingDeg = 0.0f;
  float baroAltitudeFt = 0.0f;

  RawSensors& operator=(const volatile RawSensors& other) {
    imu = other.imu;
    gps = other.gps;
    compassHeadingDeg = other.compassHeadingDeg;
    baroAltitudeFt = other.baroAltitudeFt;
    return *this;
  }
};