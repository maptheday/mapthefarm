#pragma once

// ============================================================================
// HARDWARE IO -- the real drone: one real driver for each of the flight
// controller's plugs (see FlightIo.hpp).
//
//   HardwareIo hardware;
//   fc::begin(hardware.io());        // the fly / first_mission apps
//
// Small apps can also use the drivers directly, without starting the flight
// controller (bench_test spins single motors and reads the sensors;
// esc_calibration only needs the motors). Everything they need (pins, ESC
// pulse rate, battery divider) comes from the settings (the "wiring" and
// "battery" sections of flightsettings.json).
// ============================================================================

#include "../FlightIo.hpp"
#include "Mpu6050Imu.hpp"
#include "Bme280Altimeter.hpp"
#include "Bn880Gps.hpp"
#include "Qmc5883Compass.hpp"
#include "AdcBatterySensor.hpp"
#include "PwmMotors.hpp"
#include "CrsfRadio.hpp"

struct HardwareIo {
  Mpu6050Imu       imu;
  Bme280Altimeter  altimeter;
  Bn880Gps         gps;
  Qmc5883Compass   compass;
  AdcBatterySensor battery;
  PwmMotors        motors;
  CrsfRadio        radio;

  // All seven plugs, ready for fc::begin().
  FlightIo io() {
    FlightIo io;
    io.imu       = &imu;
    io.altimeter = &altimeter;
    io.gps       = &gps;
    io.compass   = &compass;
    io.battery   = &battery;
    io.motors    = &motors;
    io.radio     = &radio;
    return io;
  }
};
