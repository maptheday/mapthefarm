#pragma once

// ============================================================================
// RC INPUT service -- the pilot's radio (ELRS/CRSF), two switches only.
//   Ch5 (START): edge-triggered -> arm+takeoff (from PARKED/LANDED) or start
//                mission (from HOLD).
//   Ch6 (STOP):  level-triggered -> emergency stop, motors cut, any phase.
//
// crsfHandleStart()/crsfHandleStop() are the "what the pilot asked for"
// handlers. They're shared: the real crsfTask calls them from parsed radio
// frames, and (under WOKWI_SIM) the SimAdapter calls them from fake serial
// commands -- so the sim exercises the exact same intent logic as hardware.
// ============================================================================

#include <Arduino.h>
#include "../state/FlightConfig.hpp"        // CRSF_* pins/channels/thresholds
#include "../state/PhaseState.hpp"          // shared, withMutex
#include "../services/Log.hpp"              // logLine
#include "../phases/PhaseSwitch.hpp"        // transitionTo

// STOP switch: cut motors unless already on the ground.
inline void crsfHandleStop() {
  FlightPhase phase;
  withMutex([&]() { phase = shared.phase; });
  if (phase != PHASE_PARKED && phase != PHASE_LANDED) {
    logLine("[CRSF] STOP switch -- emergency stop.");
    transitionTo(PHASE_PARKED, REASON_EMERGENCY_STOP);
  }
}

// START switch: arm+takeoff from the ground, or start the mission from HOLD.
inline void crsfHandleStart() {
  FlightPhase phase;
  bool hasFix;
  withMutex([&]() { phase = shared.phase; hasFix = shared.raw.gps.fix; });

  // LANDED re-arms like PARKED: motors are already confirmed off in both, so a
  // landed drone isn't "busy" -- it can fly again without a separate reset.
  if (phase == PHASE_PARKED || phase == PHASE_LANDED) {
    if (!hasFix) {
      logLine("[CRSF] START ignored -- no GPS fix.");
    } else {
      logLine("[CRSF] START switch -- arming and taking off.");
      transitionTo(PHASE_RAISE, REASON_OPERATOR_START);
    }
  } else if (phase == PHASE_HOLD) {
    logLine("[CRSF] START switch -- starting waypoint mission.");
    transitionTo(PHASE_MISSION, REASON_OPERATOR_START);
  } else {
    logLine("[CRSF] START ignored -- already flying/busy (phase: "
            + String(phaseName(phase)) + ")");
  }
}

#ifndef WOKWI_SIM
// --- Real radio only: parse CRSF frames off the wire ---
// A CRSF frame every ~4 ms: [0]=sync 0xC8, [1]=payload len, [2]=type
// (0x16 = RC channels packed), [3..]=16 channels packed as 11-bit values,
// last byte = CRC8. Raw channel range 172..1811, midpoint 992.
inline uint16_t crsfChannel(const uint8_t* payload, int chIdx) {
  int      bitOffset = chIdx * 11;
  int      byteIdx   = bitOffset / 8;
  int      bitIdx    = bitOffset % 8;
  uint32_t raw = ((uint32_t)payload[byteIdx])
               | ((uint32_t)payload[byteIdx + 1] << 8)
               | ((uint32_t)payload[byteIdx + 2] << 16);
  return (raw >> bitIdx) & 0x7FF;
}

inline void crsfTask(void* parameter) {
  logLine("[CRSF] WARNING: CRSF_RX_PIN (GPIO" + String(CRSF_RX_PIN) + ") is a placeholder -- "
          "verify it doesn't collide with EspESC.hpp's pins before first flight.");
  Serial1.begin(CRSF_BAUD, SERIAL_8N1, CRSF_RX_PIN, -1 /* TX unused */);
  logLine("[CRSF] Listening -- Ch5=START, Ch6=STOP");

  uint8_t buf[64];
  int     bufLen        = 0;
  bool    prevStartHigh = false; // for edge detection on START channel

  for (;;) {
    while (Serial1.available()) {
      uint8_t b = Serial1.read();

      // Wait for CRSF sync byte before starting a frame
      if (bufLen == 0 && b != 0xC8) continue;
      buf[bufLen++] = b;

      if (bufLen < 3) continue;

      int frameLen = buf[1] + 2; // payload length + 2 header bytes

      // Overflow guard: if we somehow accumulated garbage, reset
      if (bufLen > frameLen || bufLen >= (int)sizeof(buf)) {
        bufLen = 0;
        continue;
      }

      if (bufLen < frameLen) continue; // frame not complete yet

      // We have a full frame -- process it
      if (buf[2] == 0x16 && frameLen == 26) {
        const uint8_t* payload = buf + 3; // payload starts at byte 3

        uint16_t startVal = crsfChannel(payload, CRSF_START_CH);
        uint16_t stopVal  = crsfChannel(payload, CRSF_STOP_CH);

        // STOP: level-triggered, highest priority -- any low frame cuts motors.
        if (stopVal < CRSF_LOW_THRESHOLD) {
          crsfHandleStop();
        }

        // START: edge-triggered (only fires on the LOW->HIGH crossing).
        bool startHigh = (startVal > CRSF_HIGH_THRESHOLD);
        if (startHigh && !prevStartHigh) {
          crsfHandleStart();
        }
        prevStartHigh = startHigh;
      }

      bufLen = 0; // done with this frame, reset for next
    }
    vTaskDelay(pdMS_TO_TICKS(2)); // yield; 2 ms is well within the 4 ms frame interval
  }
}
#endif // !WOKWI_SIM
