#pragma once
#include "MissionCommander.hpp"

// ============================================================
//  WindEvent  –  describes one burst of wind
//
//  Fires when TWO conditions are both true:
//    1. The mission is in the specified stage
//    2. The time elapsed IN that stage >= stageTimeSec
//
//  This way wind events are decoupled from raw cycle numbers.
//  It doesn't matter how long the countdown takes — a gust
//  scheduled for "5 seconds into HOVER" always fires 5 seconds
//  into HOVER, every time.
//
//  Examples:
//    { Stage::TAKEOFF, 1.0,  -15.0,  0.0 }   left gust 1s after liftoff
//    { Stage::HOVER,   2.0,   0.0,  12.0 }   nose-up gust 2s into hover
//    { Stage::HOVER,   4.0,  10.0,  -8.0 }   diagonal gust 4s into hover
// ============================================================

struct WindEvent {
    MissionCommander::Stage stage;  // Which mission stage to fire in
    double stageTimeSec;            // Seconds into that stage to fire
    double rollDeg;                 // Roll displacement  (+ = right, - = left)
    double pitchDeg;                // Pitch displacement (+ = nose-up, - = nose-down)
    bool   fired = false;           // Internal: prevents double-firing
};