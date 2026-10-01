#pragma once

// ============================================================================
// SIM IMU -- the sim's IImu: reports the world's TRUE tilt and heading,
// as if from a perfect MPU6050.
// ============================================================================

#include "flight/FlightIo.hpp"   // IImu
#include "SimWorld.hpp"

class SimImu : public IImu {
public:
  SimImu(SimWorld* world) : world_(world) {}
  RawImuReading read(float) override {
    Truth t = world_->truth();
    RawImuReading r;
    r.gyroX      = t.rollDeg;      // fused roll  (the gyroX naming quirk)
    r.gyroY      = t.pitchDeg;     // fused pitch
    r.gyroZ      = t.headingDeg;   // fused yaw
    r.yawRateDps = t.yawRateDps;
    return r;
  }
private:
  SimWorld* world_;
};
