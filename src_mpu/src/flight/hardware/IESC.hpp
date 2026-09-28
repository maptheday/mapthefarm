#pragma once

// ============================================================
//  IESC  —  Electronic Speed Controller interface
//
//  Abstracts writing motor commands so FlightControllerProgram
//  has no idea whether it's running a simulation or real PWM.
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
//  M1 = front-left   M2 = front-right
//  M3 = rear-left    M4 = rear-right
//
//  Why these directions: a spinning prop twists the body the OPPOSITE way.
//  The mixer turns the drone clockwise (heading up) by speeding up M2 + M3,
//  so M2 and M3 must spin COUNTER-clockwise. (This is the standard
//  Betaflight "props in" layout.)
//
//  All values 0.0 (stopped) to 1.0 (full throttle).
// ============================================================
class IESC {
public:
    virtual void initialize()                                      = 0;
    virtual void write(double m1, double m2, double m3, double m4) = 0;
    virtual void disarm()                                          = 0;
    
    // Getters for reading back motor values (essential for abstraction)
    // On real hardware, these might return 0 if the ESC doesn't support feedback,
    // but on simulation, they enable physics calculations.
    virtual double getMotor(int index) const                       = 0;
    
    virtual ~IESC() = default;
};