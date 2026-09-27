#pragma once

// Plain data objects representing the latest sensor measurements. The
// controller uses these types without knowing which sensor produced them.

// Latest RC stick positions, normalized. throttle is 0..1 (0=stick down,
// 0.5=centered, 1=up); roll/pitch/yaw are -1..1 (0=centered). Filled by the
// RC input service on real hardware; used only by the MANUAL phase.
struct RawSticks {
  float throttle = 0.5f;  // centered = "hold altitude"
  float roll     = 0.0f;
  float pitch    = 0.0f;
  float yaw      = 0.0f;

  RawSticks& operator=(const volatile RawSticks& o) {
    throttle = o.throttle; roll = o.roll; pitch = o.pitch; yaw = o.yaw;
    return *this;
  }
};

struct RawImuReading {
  float accX  = 0.0f;
  float accY  = 0.0f;
  float accZ  = 0.0f;
  // These fields hold Madgwick roll/pitch/yaw angles in this project.
  float gyroX = 0.0f;
  float gyroY = 0.0f;
  float gyroZ = 0.0f;
  // Yaw RATE in deg/s (+ = heading increasing, i.e. clockwise seen from above).
  // The one true rate in this struct: the gyroX/Y/Z fields above are angles.
  float yawRateDps = 0.0f;
  float temp  = 0.0f;

  RawImuReading& operator=(const volatile RawImuReading& other) {
    accX = other.accX; accY = other.accY; accZ = other.accZ;
    gyroX = other.gyroX; gyroY = other.gyroY;
    gyroZ = other.gyroZ; yawRateDps = other.yawRateDps; temp = other.temp;
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

// Battery health, from best to worst. WARNING or CRITICAL makes the battery
// failsafe land the drone right where it is.
enum BatteryState : uint8_t {
  BATTERY_OK       = 0,
  BATTERY_WARNING  = 1,
  BATTERY_CRITICAL = 2
};

// The Battery service's latest view of the flight pack (see Battery.hpp).
struct RawBattery {
  bool    present   = false;   // a pack voltage is being measured (divider wired, pack plugged in)
  float   packVolts = 0.0f;    // filtered pack voltage
  float   cellVolts = 0.0f;    // filtered average voltage per cell
  float   amps      = 0.0f;    // ESTIMATED current, from the motor commands
  float   mAhUsed   = 0.0f;    // ESTIMATED charge used since the pack was plugged in
  uint8_t state     = BATTERY_OK;

  RawBattery& operator=(const volatile RawBattery& o) {
    present = o.present; packVolts = o.packVolts; cellVolts = o.cellVolts;
    amps = o.amps; mAhUsed = o.mAhUsed; state = o.state;
    return *this;
  }
};

struct RawSensors {
  RawImuReading imu;
  RawGpsReading gps;
  RawBattery    battery;
  float compassHeadingDeg = 0.0f;
  float baroAltitudeFt = 0.0f;

  RawSensors& operator=(const volatile RawSensors& other) {
    imu = other.imu;
    gps = other.gps;
    battery = other.battery;
    compassHeadingDeg = other.compassHeadingDeg;
    baroAltitudeFt = other.baroAltitudeFt;
    return *this;
  }
};