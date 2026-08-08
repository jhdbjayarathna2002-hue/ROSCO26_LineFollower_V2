// ============================================================
// PID.cpp  —  Discrete PID controller
// ============================================================
// See PID.h for full documentation.
// ============================================================

#include "PID.h"

// ============================================================
// Construction
// ============================================================

PID::PID()
    : _kp(KP), _ki(KI), _kd(KD),
      _integral(0.0f), _lastError(0.0f),
      _filteredDeriv(0.0f), _lastOutput(0.0f),
      _p(0.0f), _i(0.0f), _d(0.0f)
{
}

// ============================================================
// setGains()
// ============================================================

void PID::setGains(float kp, float ki, float kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

// ============================================================
// reset()
// Clear integral accumulator, derivative state, and last error.
// Call when:
//   - robot starts
//   - calibration finishes
//   - line is reacquired after a long loss
//   - major state transition
// ============================================================

void PID::reset()
{
    _integral      = 0.0f;
    _lastError     = 0.0f;
    _filteredDeriv = 0.0f;
    _lastOutput    = 0.0f;
    _p             = 0.0f;
    _i             = 0.0f;
    _d             = 0.0f;
}

// ============================================================
// compute()
// Returns the PID correction value.
// error : position error (positive = line right of centre)
// dt    : time elapsed since last call (seconds)
// ============================================================

float PID::compute(float error, float dt)
{
    if (dt <= 0.0f) dt = 0.001f;   // Guard against zero-dt

    // ---- Proportional ----------------------------------------
    _p = _kp * error;

    // ---- Integral with anti-windup clamp --------------------
    _integral += error * dt;
    _integral  = constrain(_integral, -PID_INTEGRAL_LIMIT, +PID_INTEGRAL_LIMIT);
    _i = _ki * _integral;

    // ---- Derivative with EMA smoothing ----------------------
    // EMA smoothing prevents derivative spikes on noisy sensor data.
    float rawDeriv = (error - _lastError) / dt;
    _filteredDeriv = PID_DERIV_EMA * rawDeriv
                   + (1.0f - PID_DERIV_EMA) * _filteredDeriv;
    _d = _kd * _filteredDeriv;

    _lastError  = error;
    _lastOutput = _p + _i + _d;

    return _lastOutput;
}
