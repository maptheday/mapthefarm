#pragma once

// ============================================================================
// RC INPUT service -- the pilot's radio (ELRS/CRSF), two switches only.
//   Ch5 (START): edge-triggered -> arm+takeoff (from PARKED/LANDED) or start
//                mission (from HOLD).
//   Ch6 (STOP):  level-triggered -> emergency stop, motors cut, any phase.
//
// crsfHandleStart()/Stop()/Land()/ManualOn()/ManualOff() are the "what the
// pilot asked for" handlers. They're the same functions behind the public API
// (fc::start(), fc::land(), ...): the real crsfTask calls them from parsed
// radio frames, and apps like the sim call them through the API -- so every
// app exercises the exact same intent logic as the real radio.
// ============================================================================

#include <Arduino.h>
#include "../state/FlightSettings.hpp"        // settings().radio, settings().wiring, CRSF_*
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
  RawBattery batt;
  withMutex([&]() { phase = shared.phase; hasFix = shared.raw.gps.fix; batt = shared.raw.battery; });

  // LANDED re-arms like PARKED: motors are already confirmed off in both, so a
  // landed drone isn't "busy" -- it can fly again without a separate reset.
  if (phase == PHASE_PARKED || phase == PHASE_LANDED) {
    if (!hasFix) {
      logLine("[CRSF] START ignored -- no GPS fix.");
    } else if (batt.state != BATTERY_OK) {
      logLine("[CRSF] START ignored -- battery low. Swap in a charged pack.");
    } else if (batt.present && batt.cellVolts < settings().battery.cellTakeoffMinV) {
      // Motors are off here, so this is the resting voltage.
      logLine("[CRSF] START ignored -- battery too low to take off (" +
              String(batt.cellVolts, 2) + " V/cell). Charge it first.");
    } else {
      logLine("[CRSF] START switch -- arming and taking off.");
      transitionTo(PHASE_RAISE, REASON_OPERATOR_START);
    }
  } else if (phase == PHASE_HOLD) {
    if (batt.state != BATTERY_OK) {
      logLine("[CRSF] START ignored -- battery low, not starting the mission.");
      return;
    }
    logLine("[CRSF] START switch -- starting waypoint mission.");
    transitionTo(PHASE_MISSION, REASON_OPERATOR_START);
  } else {
    logLine("[CRSF] START ignored -- already flying/busy (phase: "
            + String(phaseName(phase)) + ")");
  }
}

// LAND switch flipped UP: land right where it is, from anything that's flying.
// The gentle way down (STOP just cuts the motors, which drops the drone).
inline void crsfHandleLand() {
  FlightPhase phase;
  withMutex([&]() { phase = shared.phase; });
  bool airborne = phase == PHASE_RAISE || phase == PHASE_HOLD || phase == PHASE_MISSION ||
                  phase == PHASE_MANUAL || phase == PHASE_HOVER_SETTLE ||
                  phase == PHASE_RTL_CLIMB || phase == PHASE_RTL_RETURN || phase == PHASE_RTL_SETTLE;
  if (airborne) {
    logLine("[CRSF] LAND switch -- landing here.");
    transitionTo(PHASE_LANDING, REASON_OPERATOR_LAND);
  } else {
    logLine("[CRSF] LAND ignored (phase: " + String(phaseName(phase)) + ")");
  }
}

// MANUAL switch flipped ON: pilot takes the sticks. Allowed from the ground
// (bench/hand testing) or from a stable auto-hover (take over mid-flight).
inline void crsfHandleManualOn() {
  FlightPhase phase;
  withMutex([&]() { phase = shared.phase; });
  if (phase == PHASE_PARKED || phase == PHASE_LANDED || phase == PHASE_HOLD) {
    logLine("[CRSF] MANUAL switch -- pilot has the sticks.");
    transitionTo(PHASE_MANUAL, REASON_MANUAL_ON);
  } else {
    logLine("[CRSF] MANUAL ignored (phase: " + String(phaseName(phase)) + ")");
  }
}

// MANUAL switch flipped OFF: hand back to auto-hover (HOLD). STOP is still the
// hard kill; this is the "let go of the sticks safely" path.
inline void crsfHandleManualOff() {
  FlightPhase phase;
  withMutex([&]() { phase = shared.phase; });
  if (phase == PHASE_MANUAL) {
    logLine("[CRSF] MANUAL switch off -- handing back to auto-hover.");
    transitionTo(PHASE_HOLD, REASON_MANUAL_OFF);
  }
}

// Convert a raw CRSF channel (172..1811, mid 992) to a signed -1..1 deflection.
inline float crsfNorm(uint16_t raw) {
  float v = ((float)raw - CRSF_RAW_MID) / (float)(CRSF_RAW_MAX - CRSF_RAW_MID);
  if (v >  1.0f) v =  1.0f;
  if (v < -1.0f) v = -1.0f;
  return v;
}

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
  logLine("[CRSF] Receiver on GPIO" + String(settings().wiring.radioRx) + " -- check your wiring matches "
          "(ESCs use GPIO4-7, set in Motors.hpp).");
  Serial1.begin(CRSF_BAUD, SERIAL_8N1, settings().wiring.radioRx, -1 /* TX unused */);
  logLine("[CRSF] Listening -- sticks + START/STOP/MANUAL/LAND switches");

  uint8_t buf[64];
  int     bufLen         = 0;
  bool    prevStartHigh  = false; // for edge detection on START channel
  bool    prevManualHigh = false; // for edge detection on MANUAL channel
  bool    prevLandHigh   = false; // for edge detection on LAND channel

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

        uint16_t startVal  = crsfChannel(payload, settings().radio.start);
        uint16_t stopVal   = crsfChannel(payload, settings().radio.stop);
        uint16_t manualVal = crsfChannel(payload, settings().radio.manual);
        uint16_t landVal   = crsfChannel(payload, settings().radio.land);

        // Latch the four sticks so the MANUAL phase can read them. throttle is
        // published 0..1 (down..up); roll/pitch/yaw as -1..1 (centered = 0).
        float roll     = crsfNorm(crsfChannel(payload, settings().radio.roll));
        float pitch    = crsfNorm(crsfChannel(payload, settings().radio.pitch));
        float yaw      = crsfNorm(crsfChannel(payload, settings().radio.yaw));
        float throttle = (crsfNorm(crsfChannel(payload, settings().radio.throttle)) + 1.0f) * 0.5f;
        withMutex([&]() {
          shared.sticks.roll     = roll;
          shared.sticks.pitch    = pitch;
          shared.sticks.yaw      = yaw;
          shared.sticks.throttle = throttle;
          shared.rcLastFrameMs   = millis();   // the radio link is alive (Failsafes.hpp)
        });

        // STOP: level-triggered, highest priority -- any low frame cuts motors.
        if (stopVal < settings().radio.lowThreshold) {
          crsfHandleStop();
        }

        // START: edge-triggered (only fires on the LOW->HIGH crossing).
        bool startHigh = (startVal > settings().radio.highThreshold);
        if (startHigh && !prevStartHigh) {
          crsfHandleStart();
        }
        prevStartHigh = startHigh;

        // MANUAL: edge-triggered both ways -- ON grabs the sticks, OFF hands
        // back to auto-hover.
        bool manualHigh = (manualVal > settings().radio.highThreshold);
        if (manualHigh && !prevManualHigh)      crsfHandleManualOn();
        else if (!manualHigh && prevManualHigh) crsfHandleManualOff();
        prevManualHigh = manualHigh;

        // LAND: edge-triggered (only fires on the LOW->HIGH crossing).
        bool landHigh = (landVal > settings().radio.highThreshold);
        if (landHigh && !prevLandHigh) crsfHandleLand();
        prevLandHigh = landHigh;
      }

      bufLen = 0; // done with this frame, reset for next
    }
    vTaskDelay(pdMS_TO_TICKS(2)); // yield; 2 ms is well within the 4 ms frame interval
  }
}
