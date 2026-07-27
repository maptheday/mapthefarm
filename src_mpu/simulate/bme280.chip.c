// BME280 custom chip for Wokwi — compatible with Adafruit_BME280 library
// Adafruit reads each calibration register individually via separate I2C transactions
// Copyright DTViMS 2024 — fixed for Adafruit by Zach

#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Real calibration values from a BME280 chip
// These are used by the Adafruit library to compensate raw ADC values
// dig_T1=27504, dig_T2=26435, dig_T3=50
// dig_P1=36477, dig_P2=-10685, dig_P3=3024, dig_P4=7864
// dig_P5=-112, dig_P6=-7, dig_P7=9900, dig_P8=-10230, dig_P9=4285
// dig_H1=75

static const uint8_t CALIB_REGS[128] = {0}; // indexed by reg - 0x80

// Calibration lookup table — index is (register - 0x80)
// We'll fill this in chip_init
typedef struct {
  uint32_t ADDRESS_attr;
  uint8_t i;
  bool read;
  uint8_t data[4];
  uint8_t resCount;
  uint8_t resData[8];
  uint8_t calib[128]; // calib[reg - 0x80] = value
} chip_state_t;

static bool on_i2c_connect(void *user_data, uint32_t address, bool connect);
static uint8_t on_i2c_read(void *user_data);
static bool on_i2c_write(void *user_data, uint8_t data);
static void on_i2c_disconnect(void *user_data);

void chip_init() {
  chip_state_t *chip = malloc(sizeof(chip_state_t));
  memset(chip, 0, sizeof(chip_state_t));
  chip->ADDRESS_attr = attr_init("threshold", 0x76);

  // ── Calibration data (real BME280 values) ──────────────────────────────
  // dig_T1 = 27504 = 0x6B70  little-endian: [0x70, 0x6B]
  chip->calib[0x88 - 0x80] = 0x70; // T1 LSB
  chip->calib[0x89 - 0x80] = 0x6B; // T1 MSB
  // dig_T2 = 26435 = 0x6743  little-endian: [0x43, 0x67]
  chip->calib[0x8A - 0x80] = 0x43; // T2 LSB
  chip->calib[0x8B - 0x80] = 0x67; // T2 MSB
  // dig_T3 = 50 = 0x0032  little-endian: [0x32, 0x00]
  chip->calib[0x8C - 0x80] = 0x32; // T3 LSB
  chip->calib[0x8D - 0x80] = 0x00; // T3 MSB
  // dig_P1 = 36477 = 0x8E7D  little-endian: [0x7D, 0x8E]
  chip->calib[0x8E - 0x80] = 0x7D; // P1 LSB
  chip->calib[0x8F - 0x80] = 0x8E; // P1 MSB
  // dig_P2 = -10685 = 0xD603  little-endian: [0x03, 0xD6]
  chip->calib[0x90 - 0x80] = 0x03; // P2 LSB
  chip->calib[0x91 - 0x80] = 0xD6; // P2 MSB
  // dig_P3 = 3024 = 0x0BD0  little-endian: [0xD0, 0x0B]
  chip->calib[0x92 - 0x80] = 0xD0; // P3 LSB
  chip->calib[0x93 - 0x80] = 0x0B; // P3 MSB
  // dig_P4 = 7864 = 0x1EB8  little-endian: [0xB8, 0x1E]
  chip->calib[0x94 - 0x80] = 0xB8; // P4 LSB
  chip->calib[0x95 - 0x80] = 0x1E; // P4 MSB
  // dig_P5 = -112 = 0xFF90  little-endian: [0x90, 0xFF]
  chip->calib[0x96 - 0x80] = 0x90; // P5 LSB
  chip->calib[0x97 - 0x80] = 0xFF; // P5 MSB
  // dig_P6 = -7 = 0xFFF9  little-endian: [0xF9, 0xFF]
  chip->calib[0x98 - 0x80] = 0xF9; // P6 LSB
  chip->calib[0x99 - 0x80] = 0xFF; // P6 MSB
  // dig_P7 = 9900 = 0x26AC  little-endian: [0xAC, 0x26]
  chip->calib[0x9A - 0x80] = 0xAC; // P7 LSB
  chip->calib[0x9B - 0x80] = 0x26; // P7 MSB
  // dig_P8 = -10230 = 0xD80A  little-endian: [0x0A, 0xD8]
  chip->calib[0x9C - 0x80] = 0x0A; // P8 LSB
  chip->calib[0x9D - 0x80] = 0xD8; // P8 MSB
  // dig_P9 = 4285 = 0x10BD  little-endian: [0xBD, 0x10]
  chip->calib[0x9E - 0x80] = 0xBD; // P9 LSB
  chip->calib[0x9F - 0x80] = 0x10; // P9 MSB
  // 0xA0 unused
  // dig_H1 = 75 = 0x4B
  chip->calib[0xA1 - 0x80] = 0x4B; // H1

  // Humidity calibration 0xE1..0xE7
  // dig_H2 = 370 = 0x0172  little-endian: [0x72, 0x01]
  chip->calib[0xE1 - 0x80] = 0x72; // H2 LSB
  chip->calib[0xE2 - 0x80] = 0x01; // H2 MSB
  // dig_H3 = 0
  chip->calib[0xE3 - 0x80] = 0x00; // H3
  // dig_H4 = 312: stored as [0xE4] = H4[11:4], [0xE5] bits[3:0] = H4[3:0]
  // H4 = 312 = 0x138 → upper byte = 0x13, lower nibble = 0x8
  chip->calib[0xE4 - 0x80] = 0x13; // H4 MSB
  chip->calib[0xE5 - 0x80] = 0x08; // H4 LSB nibble | H5 LSB nibble
  // dig_H5 = 50: [0xE5] bits[7:4] = H5[3:0], [0xE6] = H5[11:4]
  chip->calib[0xE6 - 0x80] = 0x03; // H5 MSB
  // dig_H6 = 30 = 0x1E
  chip->calib[0xE7 - 0x80] = 0x1E; // H6

  const i2c_config_t i2c_config = {
    .user_data = chip,
    .address = 0x76,
    .scl = pin_init("SCL", INPUT),
    .sda = pin_init("SDA", INPUT),
    .connect = on_i2c_connect,
    .read = on_i2c_read,
    .write = on_i2c_write,
    .disconnect = on_i2c_disconnect,
  };
  i2c_init(&i2c_config);
  printf("BME280 chip ready\n");
}

void resetData(chip_state_t *chS) {
  chS->i = 0;
  chS->resCount = 0;
  memset(chS->resData, 0, sizeof(chS->resData));
  memset(chS->data, 0, sizeof(chS->data));
}

bool on_i2c_connect(void *user_data, uint32_t address, bool connect) {
  chip_state_t *chS = user_data;
  chS->read = false;
  uint8_t myAddr = attr_read(chS->ADDRESS_attr);
  if (myAddr != address) return false;
  return true;
}

uint8_t on_i2c_read(void *user_data) {
  chip_state_t *chS = user_data;
  chS->read = true;
  uint8_t res = 0x00;
  uint8_t reg = chS->data[0];

  if (chS->resCount > 0) {
    // Continuation of a multi-byte read — send next byte
    chS->resCount--;
    res = chS->resData[chS->resCount];
    return res;
  }

  // First byte of a new read — figure out what register was requested
  if (reg == 0xD0) {
    // Chip ID
    res = 0x60;

  } else if (reg == 0xF3) {
    // Status — 0 means not busy
    res = 0x00;

  } else if (reg == 0xF2) {
    res = 0x02;

  } else if (reg == 0xF4) {
    res = 0x4B;

  } else if (reg == 0xF5) {
    res = 0x60;

  } else if (reg == 0xFA) {
    // Temperature — read24 reads 3 bytes MSB first
    // raw temp = 519888 → gives ~25°C with our calibration
    uint32_t raw = 519888;
    chS->resCount = 2;
    chS->resData[2] = (raw >> 16) & 0xFF; // unused slot
    chS->resData[1] = (raw >> 8) & 0xFF;
    chS->resData[0] = (raw & 0xFF);
    res = (raw >> 16) & 0xFF; // first byte out
    // Actually read24 reads buffer[0]<<16 | buffer[1]<<8 | buffer[2]
    // write_then_read sends reg, reads 3 bytes: buf[0], buf[1], buf[2]
    // So we need to send MSB first
    chS->resCount = 2;
    chS->resData[1] = (raw >> 8) & 0xFF;  // buf[1]
    chS->resData[0] = raw & 0xFF;         // buf[2]
    res = (raw >> 16) & 0xFF;             // buf[0]

  } else if (reg == 0xF7) {
    // Pressure — read24, 3 bytes MSB first
    // raw press = 415148 → ~1013 hPa with our calibration
    uint32_t raw = 415148;
    chS->resCount = 2;
    chS->resData[1] = (raw >> 8) & 0xFF;
    chS->resData[0] = raw & 0xFF;
    res = (raw >> 16) & 0xFF;

  } else if (reg == 0xFD) {
    // Humidity — read16, 2 bytes MSB first
    uint16_t raw = 28445;
    chS->resCount = 1;
    chS->resData[0] = raw & 0xFF;
    res = (raw >> 8) & 0xFF;

  } else if (reg >= 0x80 && reg <= 0xFF) {
    // Calibration register — return from lookup table
    // Adafruit uses read16_LE which calls read16 then byte-swaps
    // read16 does write_then_read(reg, 1, buf, 2) → buf[0]=first, buf[1]=second
    // then returns buf[0]<<8 | buf[1], then read16_LE swaps → (buf[1]<<8 | buf[0])
    // So we need: buf[0] = calib[reg-0x80], buf[1] = calib[reg+1-0x80]
    res = chS->calib[reg - 0x80];      // first byte
    if (reg < 0xFF) {
      chS->resCount = 1;
      chS->resData[0] = chS->calib[reg + 1 - 0x80]; // second byte
    }
  }

  chS->i = 0;
  return res;
}

bool on_i2c_write(void *user_data, uint8_t data) {
  chip_state_t *chS = user_data;
  chS->data[chS->i] = data;
  chS->i++;

  if (chS->i == 2 && chS->data[0] == 0xE0 && chS->data[1] == 0xB6) {
    // Soft reset
    resetData(chS);
  } else if (chS->i == 2 && (chS->data[0] == 0xF2 ||
                              chS->data[0] == 0xF4 ||
                              chS->data[0] == 0xF5)) {
    resetData(chS);
  }
  return true;
}

void on_i2c_disconnect(void *user_data) {
  chip_state_t *chS = user_data;
  if (chS->read) {
    resetData(chS);
  }
}