#pragma once

// ============================================================================
// HARDWARE IO -- what gets plugged into each of the flight controller's
// plugs (see FlightIo.hpp), in one place:
//
//   fc::begin(realHardware());        // the real drone (drivers in real/)
//   simHardware(world, radio)         // the sim app's fakes (in sim/)
//
// Small apps can also use the parts directly, without starting the flight
// controller (bench_test spins single motors and reads the sensors). Every
// driver takes its pins from the settings (the "wiring" and "battery"
// sections of flightsettings.json), so load those first.
// ============================================================================

#include "../FlightIo.hpp"
#include "real/Mpu6050Imu.hpp"
#include "real/Bme280Altimeter.hpp"
#include "real/Bn880Gps.hpp"
#include "real/Qmc5883Compass.hpp"
#include "real/AdcBatterySensor.hpp"
#include "real/PwmMotors.hpp"
#include "real/CrsfRadio.hpp"
#include "sim/SimImu.hpp"
#include "sim/SimAltimeter.hpp"
#include "sim/SimGps.hpp"
#include "sim/SimCompass.hpp"
#include "sim/SimBatterySensor.hpp"
#include "sim/SimMotors.hpp"
#include "sim/SimRadio.hpp"

// All seven real parts. (Nothing is switched on yet: fc::begin() calls each
// part's begin(), or an app does it itself.)
inline FlightIo realHardware() {
  FlightIo io;
  io.imu       = new Mpu6050Imu();
  io.altimeter = new Bme280Altimeter();
  io.gps       = new Bn880Gps();
  io.compass   = new Qmc5883Compass();
  io.battery   = new AdcBatterySensor();
  io.motors    = new PwmMotors();
  io.radio     = new CrsfRadio();
  return io;
}

// All seven sim parts: fakes that read the simulated world. The radio is
// passed in because the scenario's pilot holds that same radio.
inline FlightIo simHardware(SimWorld* world, SimRadio* radio) {
  FlightIo io;
  io.imu       = new SimImu(world);
  io.altimeter = new SimAltimeter(world);
  io.gps       = new SimGps(world);
  io.compass   = new SimCompass(world);
  io.battery   = new SimBatterySensor(world);
  io.motors    = new SimMotors(world);
  io.radio     = radio;
  return io;
}
