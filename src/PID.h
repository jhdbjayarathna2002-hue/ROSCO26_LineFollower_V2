// ============================================================
// PID.h  —  Discrete PID controller
// ============================================================
// Standard proportional-integral-derivative controller.
// Used for line-following position error correction.
//
// Error convention:
//   error = 0   -> line is centred
//   error < 0   -> line is LEFT  of centre
//   error > 0   -> line is RIGHT of centre
//
// Output:
//   correction = P + I + D
//   leftSpeed  = baseSpeed - correction
//   rightSpeed = baseSpeed + correction
//
// Features:
//   Integral anti-windup (clamp)
//   Derivative EMA smoothing
//   Reset on state transitions
// ============================================================

#pragma once
#include <Arduino.h>
#include "Config.h"

class PID
{
public:
    // ---- Construction ----------------------------------------
    PID();

    // ---- Configuration ---------------------------------------
    void setGains(float kp, float ki, float kd);

    // ---- Control ---------------------------------------------

    // Compute PID output from current error.
    // dt: elapsed time since last call in seconds.
    // Returns the correction value.
    float compute(float error, float dt);

    // Reset internal state (integral, last error, EMA).
    // Call when:
    //   - robot starts
    //   - calibration finishes
    //   - line is reacquired after a long loss
    //   - major state transition
    void reset();

    // ---- State accessors (for debug) -------------------------
    float getP()           const { return _p; }
    float getI()           const { return _i; }
    float getD()           const { return _d; }
    float getIntegral()    const { return _integral; }
    float getLastError()   const { return _lastError; }
    float getLastOutput()  const { return _lastOutput; }

private:
    float _kp;
    float _ki;
    float _kd;

    float _integral;
    float _lastError;       // raw previous error (for derivative)
    float _filteredDeriv;   // EMA-smoothed derivative
    float _lastOutput;

    // Decomposed terms (for debug)
    float _p;
    float _i;
    float _d;
};
