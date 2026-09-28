#pragma once

// ============================================================================
// FLIGHT IO -- the flight controller's plugs: one small interface per sensor,
// one for the motors, one for the radio. This is the injection point of the
// public API (like constructor injection in C#).
//
//   real drone:  HardwareIo (flight/hardware/HardwareIo.hpp) holds one real
//                driver per plug: MPU6050, BME280, GPS, compass, battery pin,
//                the 4 ESCs, the CRSF receiver.
//   sim:         the sim app (apps/sim) plugs in fake sensors that read a
//                simulated world, and a fake radio held by a fake pilot. A
//                scenario can wrap any one plug to make it misbehave (a GPS
//                that loses its fix, a radio that goes quiet).
//
// The flight controller only ever sees these interfaces. It can't tell a real
// GPS from a simulated one, or from a simulated one that's been broken on
// purpose.
//
// Who calls what:
//   begin()      once, from fc::begin(), before the flight loops start
//   IImu, IAltimeter, IBatterySensor   every physics tick (200 Hz)
//   IGps, ICompass                     every nav tick (10 Hz)
//   IRadio                             every 2 ms, by the flight controller's radio loop
//   IMotors                            from the Motors service, every physics tick
// ============================================================================

#include "models/SensorTypes.hpp"    // RawImuReading, RawGpsReading, RawSticks
#include "models/ControlTypes.hpp"   // MotorMix

// Attitude: fused roll/pitch/yaw (stored in gyroX/Y/Z, a naming quirk) plus
// the yaw rate. `dt` is the seconds since the last read, for the fusion filter.
class IImu {
public:
  virtual ~IImu() {}
  virtual void begin() {}
  virtual RawImuReading read(float dt) = 0;
};

// Height above the launch point, in feet.
class IAltimeter {
public:
  virtual ~IAltimeter() {}
  virtual void begin() {}
  virtual float readFt() = 0;
};

// Position. fix = false when there's no usable fix (the other fields are then
// ignored).
class IGps {
public:
  virtual ~IGps() {}
  virtual void begin() {}
  virtual RawGpsReading read() = 0;
};

// Heading, 0..360 degrees, clockwise from north. The calibration methods are
// used by the CALIBRATE phase; a compass that can't be calibrated (a fake one)
// just ignores them.
class ICompass {
public:
  virtual ~ICompass() {}
  virtual void begin() {}
  virtual float readHeadingDeg() = 0;
  virtual void startCalibration() {}
  virtual void sampleCalibration() {}
  virtual void finishCalibration() {}
};

// The flight pack's voltage, in volts. 0 when there's nothing to measure.
class IBatterySensor {
public:
  virtual ~IBatterySensor() {}
  virtual void begin() {}
  virtual float readPackVolts() = 0;
};

// The 4 motors. Throttle 0.0 (stopped) .. 1.0 (full). Motor numbers are 1..4.
//
//  Motor layout (top-down view, spin direction seen from ABOVE):
//
//         FRONT
//    M1(CW)   M2(CCW)
//       \      /
//        \    /
//        /    \
//       /      \
//    M3(CCW)  M4(CW)
//         REAR
//
//  Why these directions: a spinning prop twists the body the OPPOSITE way.
//  The mixer turns the drone clockwise (heading up) by speeding up M2 + M3,
//  so M2 and M3 must spin COUNTER-clockwise. (This is the standard
//  Betaflight "props in" layout.)
class IMotors {
public:
  virtual ~IMotors() {}
  virtual void begin() {}
  virtual void write(const MotorMix& mix) = 0;
  virtual void stop() = 0;
  virtual void writeOne(int motor, float throttle) = 0;   // bench testing, PROPS OFF
  virtual void stopOne(int motor) = 0;
};

// One radio frame, already decoded: where the sticks and switches are.
struct RadioFrame {
  RawSticks sticks;            // throttle 0..1, roll/pitch/yaw -1..1
  bool      start  = false;    // START switch up   (edge: take off / start the mission)
  bool      stop   = false;    // STOP engaged      (level: motors off, every frame it's on)
  bool      manual = false;    // MANUAL switch up  (edges: sticks on / off)
  bool      land   = false;    // LAND switch up    (edge: land where it is)
};

// The pilot's radio receiver. read() returns true and fills `out` when a new
// frame has arrived since the last call, false when there's nothing new. The
// flight controller does the rest: switch edges, the sticks, and the
// radio-loss failsafe (no frame for a while = no pilot).
class IRadio {
public:
  virtual ~IRadio() {}
  virtual void begin() {}
  virtual bool read(RadioFrame& out) = 0;
};

// Everything the flight controller plugs into. `radio` may be null: then the
// drone has no pilot, so it never takes off on its own (compass_calibration).
struct FlightIo {
  IImu*           imu       = nullptr;
  IAltimeter*     altimeter = nullptr;
  IGps*           gps       = nullptr;
  ICompass*       compass   = nullptr;
  IBatterySensor* battery   = nullptr;
  IMotors*        motors    = nullptr;
  IRadio*         radio     = nullptr;
};

// The plugs the flight controller was started with (filled in by fc::begin()).
// The Motors service and the CALIBRATE phase reach their plug through this.
inline FlightIo& flightIo() { static FlightIo io; return io; }
