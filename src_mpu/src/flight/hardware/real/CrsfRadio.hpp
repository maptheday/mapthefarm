#pragma once

// ============================================================================
// CRSF RADIO -- the real drone's IRadio: listens to the radio receiver and
// reports where your sticks and switches are.
//
// --- Start from an RC car --------------------------------------------------
// An RC car's remote keeps shouting, over and over, "the steering knob is
// HERE, the trigger is HERE". The car's receiver hears that and turns the
// wheels. A drone radio is the same idea, just with more knobs:
//
//   your transmitter ~~~ radio ~~~> ELRS receiver (on the drone) ──wire──> ESP32 (this code)
//
// About 250 times a second the receiver sends the ESP32 a "postcard" (a CRSF
// frame) saying where every stick and switch is right now. Each stick or
// switch is one CHANNEL, numbered like your radio's screen: CH1..CH16. The
// settings ("radio.channels" in flightsettings.json) say which is which --
// e.g. CH3 = throttle stick, CH6 = STOP switch.
//
// --- The library does the hard part ----------------------------------------
// Reading those postcards byte by byte (and checking each one isn't garbled)
// is done by the AlfredoCRSF library (added in platformio.ini, like a NuGet
// package). We just ask it "where's channel 3?" and get a number in
// microseconds, the standard RC scale:
//
//   stick all the way one way   -> ~1000
//   stick centered              ->  1500
//   stick all the way the other -> ~2000
//   a switch: down ~1000, middle 1500, up ~2000
//
// What a switch flip MEANS (take off? land?) is not decided here: this file
// only reports positions. The flight controller decides (FlightController.hpp
// and services/RcInput.hpp), the same way for this radio and the sim's fake one.
// ============================================================================

#include <Arduino.h>
#include <AlfredoCRSF.h>                     // the CRSF library (CRSF_BAUDRATE, AlfredoCRSF)
#include "../../FlightIo.hpp"                // IRadio, RadioFrame
#include "../../state/FlightSettings.hpp"    // settings().radio, settings().wiring.radioRx
#include "../../services/Log.hpp"            // logLine

class CrsfRadio : public IRadio {
public:
  // Start listening on the wire from the receiver (420,000 bits per second).
  void begin() override {
    logLine("[CRSF] Receiver on GPIO" + String(settings().wiring.radioRx) +
            " -- check your wiring matches the settings (wiring.radioRx).");
    Serial1.begin(CRSF_BAUDRATE, SERIAL_8N1, settings().wiring.radioRx, -1 /* we only listen, never send */);
    crsf_.begin(Serial1);
    logLine("[CRSF] Listening -- sticks + START/STOP/MANUAL/LAND switches");
  }

  // Called every 2 ms by the flight controller. Lets the library read any new
  // postcards, then reports the latest stick and switch positions.
  //
  // Returns false when the radio isn't being heard: the library calls the
  // link "down" once no good postcard has arrived for 300 ms. The flight
  // controller's radio-loss failsafe takes it from there.
  bool read(RadioFrame& out) override {
    crsf_.update();
    if (!crsf_.isLinkUp()) return false;

    const auto& r = settings().radio;   // which channel is which (from flightsettings.json)

    // Sticks: roll/pitch/yaw as -1..+1 (0 = centered); throttle as 0..1 (0 = all the way down).
    out.sticks.roll     = toMinusOneToPlusOne(crsf_.getChannel(r.roll));
    out.sticks.pitch    = toMinusOneToPlusOne(crsf_.getChannel(r.pitch));
    out.sticks.yaw      = toMinusOneToPlusOne(crsf_.getChannel(r.yaw));
    out.sticks.throttle = (toMinusOneToPlusOne(crsf_.getChannel(r.throttle)) + 1.0f) * 0.5f;

    // Switches: "up" means above switchUpAbove (1700).
    out.start  = switchIsUp(r.start);
    out.manual = switchIsUp(r.manual);
    out.land   = switchIsUp(r.land);
    // STOP works the other way round: the motors may only run while its
    // switch is UP. Down, in the middle, or not set up on the radio at all
    // all mean STOP -- the safe answer.
    out.stop   = !switchIsUp(r.stop);
    return true;
  }

private:
  bool switchIsUp(int channel) { return crsf_.getChannel(channel) > settings().radio.switchUpAbove; }

  // Microseconds (1000..1500..2000) as -1 (all the way one way) .. 0 (centered)
  // .. +1 (all the way the other way).
  static float toMinusOneToPlusOne(int us) {
    float v = (us - 1500) / 500.0f;
    if (v >  1.0f) v =  1.0f;   // the very ends read ~988 / ~2012; clamp them
    if (v < -1.0f) v = -1.0f;
    return v;
  }

  AlfredoCRSF crsf_;
};
