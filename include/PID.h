#pragma once

#include <Arduino.h>
#include "Config.h"

// ============================================================
// PID CONTROLLER
// ============================================================
//
// Input:
//     Line position
//
// Output:
//     Steering correction
//
// Error convention:
//
//     Negative = line is LEFT
//     Zero     = line is CENTER
//     Positive = line is RIGHT
//
// C7 + C8 are the center sensors.
// ============================================================

class PIDController
{
public:

    // --------------------------------------------------------
    // Initialize PID
    // --------------------------------------------------------

    void begin();


    // --------------------------------------------------------
    // Calculate PID correction
    //
    // error = sensor line position
    //
    // Returns steering correction.
    // --------------------------------------------------------

    float calculate(
        float error
    );


    // --------------------------------------------------------
    // Reset PID memory
    //
    // Used when:
    // - starting a turn
    // - reacquiring the line
    // - changing track state
    // --------------------------------------------------------

    void reset();


    // --------------------------------------------------------
    // Get current PID values
    // --------------------------------------------------------

    float getIntegral();

    float getPreviousError();


private:

    float integral;

    float previousError;

    unsigned long previousTime;
};


// ============================================================
// GLOBAL PID OBJECT
// ============================================================

extern PIDController pid;