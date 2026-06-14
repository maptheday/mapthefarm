#pragma once

// ============================================================
//  PID Controller
//  Three knobs that work together to correct error:
//
//  Kp  (Proportional) – How hard to push RIGHT NOW based on
//       how far off you are. Big error = big push.
//
//  Ki  (Integral) – How hard to push based on how long the
//       error has been piling up. Fights steady-state drift.
//
//  Kd  (Derivative) – How hard to push based on how fast the
//       error is CHANGING. Acts like a brake to stop overshoot.
// ============================================================

class PIDController {
public:
    // Tune these to change how the drone responds
    double Kp = 8.0;   // Proportional gain
    double Ki = 0.1;   // Integral gain
    double Kd = 3.0;   // Derivative gain

    // Clamp the output so motors don't get crazy commands
    double maxOutput = 100.0;

    // --------------------------------------------------------
    //  calculate()
    //  target   – where we WANT to be (e.g. 0.0 degrees)
    //  measured – where we ARE       (e.g. -20.0 degrees)
    //  dt       – seconds since last call
    //  returns  – motor correction value
    // --------------------------------------------------------
    double calculate(double target, double measured, double dt) {
        double error = target - measured;

        // Integral: keep a running sum of error over time
        integral += error * dt;

        // Clamp integral to prevent "windup" (runaway accumulation)
        if (integral >  10.0) integral =  10.0;
        if (integral < -10.0) integral = -10.0;

        // Derivative: rate of change of error
        double derivative = (error - previousError) / dt;
        previousError = error;

        // Final output = sum of all three terms
        double output = (Kp * error) + (Ki * integral) + (Kd * derivative);

        // Clamp output to maxOutput
        if (output >  maxOutput) output =  maxOutput;
        if (output < -maxOutput) output = -maxOutput;

        return output;
    }

    void reset() {
        integral = 0.0;
        previousError = 0.0;
    }

private:
    double integral      = 0.0;
    double previousError = 0.0;
};