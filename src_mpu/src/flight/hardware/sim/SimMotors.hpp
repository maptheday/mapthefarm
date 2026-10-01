#pragma once

// ============================================================================
// SIM MOTORS -- the sim's IMotors: hands the motor commands to the world,
// whose physics turns them into thrust and movement.
// ============================================================================

#include "flight/FlightIo.hpp"   // IMotors
#include "SimWorld.hpp"

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
