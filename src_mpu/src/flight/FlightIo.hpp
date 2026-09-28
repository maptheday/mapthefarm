#pragma once

// ============================================================================
// FLIGHT IO -- where the flight controller's inputs come from, and where its
// motor commands go. This is the injection point of the public API.
//
//   real drone:  HardwareIo (flight/io/HardwareIo.hpp) reads the MPU6050,
//                BME280, GPS, compass and battery divider, and drives the ESCs.
//   sim:         SimIo (apps/sim) steps the on-chip physics and hands back
//                fake readings; motor commands feed the physics.
//
// The flight controller never knows which one it has. It calls:
//   readFast()   every physics tick (200 Hz), before the phase's physicsTick
//   readSlow()   every nav tick (10 Hz), before the phase's navTick
//   writeMotors / stopMotors / writeMotor   from the Motors service
// ============================================================================

#include "models/SensorTypes.hpp"    // RawImuReading, RawGpsReading
#include "models/ControlTypes.hpp"   // MotorMix

// Read every physics tick.
struct FastInputs {
  RawImuReading imu;              // fused roll/pitch/yaw (gyroX/Y/Z) + yaw rate
  float         baroAltitudeFt = 0.0f;
  float         packVolts      = 0.0f;   // 0 if there's no battery reading
};

// Read every nav tick.
struct SlowInputs {
  RawGpsReading gps;              // gps.fix = false when there's no usable fix
  float         compassHeadingDeg = 0.0f;
};

class FlightIo {
public:
  virtual ~FlightIo() {}

  // Bring up whatever is behind this IO. Called by fc::begin(), after the
  // settings are loaded, before the flight tasks start.
  virtual void begin() = 0;

  virtual void readFast(float dt, FastInputs& in) = 0;
  virtual void readSlow(SlowInputs& in) = 0;

  // Motor commands, 0.0 (stopped) .. 1.0 (full). Motor numbers are 1..4:
  // M1 front-left, M2 front-right, M3 rear-left, M4 rear-right.
  virtual void writeMotors(const MotorMix& mix) = 0;
  virtual void writeMotor(int motor, float throttle) = 0;
  virtual void stopMotors() = 0;
  virtual void stopMotor(int motor) = 0;
};
