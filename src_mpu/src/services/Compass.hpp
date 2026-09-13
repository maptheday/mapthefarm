#pragma once

// ============================================================================
// COMPASS service -- owns the magnetometer hardware and its calibration.
// Two jobs:
//   1. readHeadingDeg()  -- the live heading the nav loop steers by.
//   2. calibration       -- start/sample/finish, driven by CalibratePhase.
// Calibration offsets/scales are stored in flash (Preferences) and reloaded on
// begin(), so the drone only needs calibrating once per magnetic environment.
// ============================================================================

#include <Arduino.h>
#include <QMC5883LCompass.h>
#include <Preferences.h>

class Compass {
public:
  // Bring up the sensor and apply any stored calibration.
  void begin() {
    hw_.init();
    hw_.setMode(0x01, 0x0C, 0x10, 0xC0);
    loadCalibration();
  }

  // Live heading in degrees (0-360). Called by the nav loop each tick.
  float readHeadingDeg() {
    hw_.read();
    return hw_.getAzimuth();
  }

  // --- Calibration, driven by CalibratePhase over ~30s ---

  // Reset the running min/max before collecting samples.
  void startCalibration() {
    minX_ = minY_ = minZ_ = 32767;
    maxX_ = maxY_ = maxZ_ = -32768;
  }

  // Take one reading and widen the min/max envelope. Call repeatedly while the
  // operator rotates the drone through all orientations.
  void sampleCalibration() {
    hw_.read();
    int16_t x = hw_.getX();
    int16_t y = hw_.getY();
    int16_t z = hw_.getZ();
    minX_ = min(minX_, x); maxX_ = max(maxX_, x);
    minY_ = min(minY_, y); maxY_ = max(maxY_, y);
    minZ_ = min(minZ_, z); maxZ_ = max(maxZ_, z);
  }

  // Turn the collected envelope into offsets (hard-iron) + scales (soft-iron),
  // apply them to the sensor, and save to flash so they survive a reboot.
  void finishCalibration() {
    float offX = (minX_ + maxX_) / 2.0f;
    float offY = (minY_ + maxY_) / 2.0f;
    float offZ = (minZ_ + maxZ_) / 2.0f;

    float rangeX = (maxX_ - minX_) / 2.0f;
    float rangeY = (maxY_ - minY_) / 2.0f;
    float rangeZ = (maxZ_ - minZ_) / 2.0f;
    float avg    = (rangeX + rangeY + rangeZ) / 3.0f;

    float sclX = (rangeX > 0) ? (avg / rangeX) : 1.0f;
    float sclY = (rangeY > 0) ? (avg / rangeY) : 1.0f;
    float sclZ = (rangeZ > 0) ? (avg / rangeZ) : 1.0f;

    hw_.setCalibration(offX, offY, offZ, sclX, sclY, sclZ);

    prefs_.begin("compass", false);
    prefs_.putFloat("offX", offX); prefs_.putFloat("offY", offY); prefs_.putFloat("offZ", offZ);
    prefs_.putFloat("sclX", sclX); prefs_.putFloat("sclY", sclY); prefs_.putFloat("sclZ", sclZ);
    prefs_.end();
  }

private:
  void loadCalibration() {
    float offX = 0, offY = 0, offZ = 0, sclX = 1, sclY = 1, sclZ = 1;
    prefs_.begin("compass", true);
    if (prefs_.isKey("offX")) {
      offX = prefs_.getFloat("offX", 0); offY = prefs_.getFloat("offY", 0); offZ = prefs_.getFloat("offZ", 0);
      sclX = prefs_.getFloat("sclX", 1); sclY = prefs_.getFloat("sclY", 1); sclZ = prefs_.getFloat("sclZ", 1);
    }
    prefs_.end();
    hw_.setCalibration(offX, offY, offZ, sclX, sclY, sclZ);
  }

  QMC5883LCompass hw_;
  Preferences     prefs_;
  int16_t minX_ = 32767, maxX_ = -32768;
  int16_t minY_ = 32767, maxY_ = -32768;
  int16_t minZ_ = 32767, maxZ_ = -32768;
};

// The one Compass instance (defined in the .ino).
extern Compass compass;
