#pragma once

// ============================================================================
// SIM SENSORS + MOTORS -- the sim's version of each of the flight controller's
// plugs (see flight/FlightIo.hpp). Each one just looks at the SimWorld and
// reports what a perfect sensor would read; the motors push the world.
//
// simHardware() in flight/hardware/HardwareIo.hpp bundles one of each.
//
// None of these know about scenarios. A scenario that wants a sensor to
// misbehave wraps it (see scenarios/GpsLoss.hpp).
// ============================================================================

#include "flight/FlightIo.hpp"
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

class SimAltimeter : public IAltimeter {
public:
  SimAltimeter(SimWorld* world) : world_(world) {}
  float readFt() override { return world_->truth().upFt; }
private:
  SimWorld* world_;
};

class SimGps : public IGps {
public:
  SimGps(SimWorld* world) : world_(world) {}
  RawGpsReading read() override {
    Truth t = world_->truth();
    RawGpsReading r;
    r.lat       = t.lat;
    r.lon       = t.lon;
    r.fix       = true;
    r.sats      = 10;
    r.lastFixMs = millis();
    return r;
  }
private:
  SimWorld* world_;
};

class SimCompass : public ICompass {
public:
  SimCompass(SimWorld* world) : world_(world) {}
  float readHeadingDeg() override { return world_->truth().headingDeg; }
private:
  SimWorld* world_;
};

class SimBatterySensor : public IBatterySensor {
public:
  SimBatterySensor(SimWorld* world) : world_(world) {}
  float readPackVolts() override { return world_->truth().packVolts; }
private:
  SimWorld* world_;
};

class SimMotors : public IMotors {
public:
  SimMotors(SimWorld* world) : world_(world) {}
  void write(const MotorMix& mix) override { world_->setMotors(mix); }
  void stop() override { world_->setMotors(MotorMix{}); }
  void writeOne(int, float) override {}   // bench-test only; not simulated
  void stopOne(int) override {}
private:
  SimWorld* world_;
};
