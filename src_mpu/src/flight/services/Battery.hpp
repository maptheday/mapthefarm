#pragma once

// ============================================================================
// BATTERY service -- "how much fuel is left?" Modeled on INAV's battery.c.
//
// Three independent gauges, because no single one is trustworthy on its own:
//   1. VOLTAGE   the pack voltage (the IBatterySensor plug reads it: on the
//                real drone through a two-resistor divider), smoothed with a 1 Hz filter so a throttle
//                punch (voltage sag) doesn't trip an alarm. LiFe voltage is
//                flat until the very end, so this is the BACKUP gauge.
//   2. mAh USED  estimated from the motor commands (INAV's "virtual current
//                sensor"): a prop's power grows with speed cubed, so
//                  amps ≈ IDLE + MOTOR_FULL × (m1³ + m2³ + m3³ + m4³)
//                added up over time. This is the MAIN gauge for LiFe.
//   3. TIME      the flight-time limit (settings().safety.maxFlightTimeMs), in Failsafes.hpp.
//
// Gauges 1 and 2 each give a state; the worse one wins:
//   OK ──► WARNING ──► CRITICAL
// Either one makes the drone LAND WHERE IT IS (see Failsafes::checkBattery in
// Failsafes.hpp). On a big field, flying home could cost more than is left.
// The voltage state uses a small buffer (hysteresis), like INAV, so it doesn't
// flicker back and forth when the voltage sits right at a threshold.
//
// If the divider isn't wired (no voltage reading), the voltage gauge is simply
// skipped: the mAh estimate and the timer still protect the pack.
// ============================================================================

#include <Arduino.h>
#include "../state/FlightSettings.hpp"   // settings().battery
#include "../models/SensorTypes.hpp"   // RawBattery, BatteryState
#include "../models/ControlTypes.hpp"  // MotorMix
#include "Log.hpp"                     // logLine

class Battery {
public:
  // Call every physics tick with the measured pack voltage and the motor mix
  // that was just sent. Updates the filtered voltage, the estimated current and
  // mAh, and the OK / WARNING / CRITICAL state.
  void update(float measuredPackVolts, const MotorMix& mix, float dt) {
    if (dt <= 0.0f || dt > 0.5f) dt = 0.005f;

    // --- 1. Is a pack connected? (INAV: "battery present" above ~2.2 V) ---
    bool nowPresent = measuredPackVolts > PRESENT_PACK_V;
    if (nowPresent && !r_.present) {
      // Just plugged in (or first reading): start the filter at the real value
      // and assume a freshly charged pack, so the mAh count starts at zero.
      r_.packVolts = measuredPackVolts;
      r_.mAhUsed   = 0.0f;
      voltState_   = BATTERY_OK;
      logLine("[BATTERY] Connected: " + String(measuredPackVolts, 2) + " V (" +
              String(measuredPackVolts / settings().battery.cells, 2) + " V/cell)");
    } else if (!nowPresent && r_.present) {
      logLine("[BATTERY] Voltage reading lost -- using the mAh estimate and timer only.");
    }
    r_.present = nowPresent;

    // --- 2. Smooth the voltage (1 Hz low-pass, like INAV's VBATT_LPF_FREQ) ---
    if (r_.present) {
      float alpha = dt / (FILTER_TAU_S + dt);
      r_.packVolts += alpha * (measuredPackVolts - r_.packVolts);
      r_.cellVolts  = r_.packVolts / settings().battery.cells;
    } else {
      r_.packVolts = 0.0f;
      r_.cellVolts = 0.0f;
    }

    // --- 3. Estimate current and add up the charge used ---
    r_.amps = settings().battery.currentIdleA +
              settings().battery.currentMotorFullA * (cube(mix.m1) + cube(mix.m2) + cube(mix.m3) + cube(mix.m4));
    r_.mAhUsed += r_.amps * dt * (1000.0f / 3600.0f);   // amps × hours → mAh

    // --- 4. Decide the state: the worse of the two gauges ---
    uint8_t oldState = r_.state;
    r_.state = worse(voltageState(), capacityState());
    if (r_.state != oldState) {
      const char* names[3] = { "OK", "WARNING", "CRITICAL" };
      logLine(String("[BATTERY] ") + names[r_.state] + " -- " +
              (r_.present ? String(r_.cellVolts, 2) + " V/cell, " : String("no voltage reading, ")) +
              String(r_.mAhUsed, 0) + " mAh used (est.)");
    }
  }

  const RawBattery& reading() const { return r_; }

private:
  static constexpr float PRESENT_PACK_V = 2.5f;    // below this: nothing connected
  static constexpr float FILTER_TAU_S   = 0.159f;  // 1 Hz cutoff: tau = 1 / (2π × 1 Hz)
  static constexpr float HYSTERESIS_V   = 0.03f;   // per cell (INAV: 0.1 V per pack)

  RawBattery r_;
  uint8_t    voltState_ = BATTERY_OK;

  static float   cube(float x)  { if (x < 0.0f) x = 0.0f; return x * x * x; }
  static uint8_t worse(uint8_t a, uint8_t b) { return a > b ? a : b; }

  // Voltage gauge, with a buffer so it doesn't flicker (INAV's state machine).
  uint8_t voltageState() {
    if (!r_.present) return BATTERY_OK;   // no reading: this gauge can't say anything
    float v = r_.cellVolts;
    switch (voltState_) {
      case BATTERY_OK:
        if (v <= settings().battery.cellWarningV - HYSTERESIS_V) voltState_ = BATTERY_WARNING;
        break;
      case BATTERY_WARNING:
        if (v <= settings().battery.cellCriticalV - HYSTERESIS_V)     voltState_ = BATTERY_CRITICAL;
        else if (v > settings().battery.cellWarningV + HYSTERESIS_V)  voltState_ = BATTERY_OK;
        break;
      case BATTERY_CRITICAL:
        if (v > settings().battery.cellCriticalV + HYSTERESIS_V)      voltState_ = BATTERY_WARNING;
        break;
    }
    return voltState_;
  }

  // Capacity gauge: the estimated charge used only ever goes up.
  uint8_t capacityState() const {
    float usedFrac = r_.mAhUsed / settings().battery.capacityMah;
    if (usedFrac >= settings().battery.criticalUsed) return BATTERY_CRITICAL;
    if (usedFrac >= settings().battery.warningUsed)  return BATTERY_WARNING;
    return BATTERY_OK;
  }
};

