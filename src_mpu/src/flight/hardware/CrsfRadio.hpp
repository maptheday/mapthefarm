#pragma once

// ============================================================================
// CRSF RADIO -- the real drone's IRadio: the ELRS receiver, speaking CRSF on
// Serial1. It turns the raw byte stream into RadioFrames (where the sticks and
// switches are). Which channel is which switch, and what counts as "up", come
// from settings().radio. What a switch flip MEANS is decided by the flight
// controller (services/RcInput.hpp), the same for this radio and the sim's.
//
// A CRSF frame arrives every ~4 ms: [0]=sync 0xC8, [1]=payload len, [2]=type
// (0x16 = RC channels packed), [3..]=16 channels packed as 11-bit values,
// last byte = CRC8. Raw channel range 172..1811, midpoint 992.
// ============================================================================

#include <Arduino.h>
#include "../FlightIo.hpp"                // IRadio, RadioFrame
#include "../state/FlightSettings.hpp"    // settings().radio, settings().wiring.radioRx
#include "../state/FlightConstants.hpp"   // CRSF_BAUD, CRSF_RAW_*
#include "../services/Log.hpp"            // logLine

class CrsfRadio : public IRadio {
public:
  void begin() override {
    logLine("[CRSF] Receiver on GPIO" + String(settings().wiring.radioRx) +
            " -- check your wiring matches the settings (wiring.radioRx).");
    Serial1.begin(CRSF_BAUD, SERIAL_8N1, settings().wiring.radioRx, -1 /* TX unused */);
    logLine("[CRSF] Listening -- sticks + START/STOP/MANUAL/LAND switches");
  }

  // Read whatever bytes have arrived. True (and `out` filled) when that
  // completed an RC-channels frame.
  bool read(RadioFrame& out) override {
    bool gotFrame = false;
    while (Serial1.available()) {
      uint8_t b = Serial1.read();

      // Wait for the CRSF sync byte before starting a frame.
      if (bufLen_ == 0 && b != 0xC8) continue;
      buf_[bufLen_++] = b;
      if (bufLen_ < 3) continue;

      int frameLen = buf_[1] + 2;   // payload length + 2 header bytes

      // Overflow guard: if we somehow accumulated garbage, start over.
      if (bufLen_ > frameLen || bufLen_ >= (int)sizeof(buf_)) { bufLen_ = 0; continue; }
      if (bufLen_ < frameLen) continue;   // frame not complete yet

      if (buf_[2] == 0x16 && frameLen == 26) {
        decode(buf_ + 3, out);   // payload starts at byte 3
        gotFrame = true;
      }
      bufLen_ = 0;   // done with this frame, reset for the next
    }
    return gotFrame;
  }

private:
  void decode(const uint8_t* payload, RadioFrame& out) {
    const auto& r = settings().radio;
    // throttle is 0..1 (down..up); roll/pitch/yaw are -1..1 (centered = 0).
    out.sticks.roll     = norm(channel(payload, r.roll));
    out.sticks.pitch    = norm(channel(payload, r.pitch));
    out.sticks.yaw      = norm(channel(payload, r.yaw));
    out.sticks.throttle = (norm(channel(payload, r.throttle)) + 1.0f) * 0.5f;
    out.stop   = channel(payload, r.stop)   < r.lowThreshold;    // STOP: switch LOW = motors off
    out.start  = channel(payload, r.start)  > r.highThreshold;
    out.manual = channel(payload, r.manual) > r.highThreshold;
    out.land   = channel(payload, r.land)   > r.highThreshold;
  }

  // One 11-bit channel out of the packed payload.
  static uint16_t channel(const uint8_t* payload, int chIdx) {
    int      bitOffset = chIdx * 11;
    int      byteIdx   = bitOffset / 8;
    int      bitIdx    = bitOffset % 8;
    uint32_t raw = ((uint32_t)payload[byteIdx])
                 | ((uint32_t)payload[byteIdx + 1] << 8)
                 | ((uint32_t)payload[byteIdx + 2] << 16);
    return (raw >> bitIdx) & 0x7FF;
  }

  // A raw channel (172..1811, mid 992) as a signed -1..1 deflection.
  static float norm(uint16_t raw) {
    float v = ((float)raw - CRSF_RAW_MID) / (float)(CRSF_RAW_MAX - CRSF_RAW_MID);
    if (v >  1.0f) v =  1.0f;
    if (v < -1.0f) v = -1.0f;
    return v;
  }

  uint8_t buf_[64];
  int     bufLen_ = 0;
};
