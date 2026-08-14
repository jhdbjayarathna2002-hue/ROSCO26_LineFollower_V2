#include "PID.h"

// ============================================================
// GLOBAL PID OBJECT
// ============================================================

PIDController pid;


// ============================================================
// BEGIN
// ============================================================

void PIDController::begin()
{
    integral = 0.0f;

    previousError = 0.0f;

    previousTime = micros();
}


// ============================================================
// CALCULATE PID
// ============================================================

float PIDController::calculate(
    float error
)
{
    unsigned long currentTime =
        micros();


    // --------------------------------------------------------
    // Calculate elapsed time
    // --------------------------------------------------------

    float dt =
        (
            currentTime -
            previousTime
        )
        /
        1000000.0f;


    previousTime =
        currentTime;


    // --------------------------------------------------------
    // Protect against invalid timing
    // --------------------------------------------------------

    if (
        dt <= 0.0f
    )
    {
        dt = 0.0025f;
    }


    if (
        dt > 0.05f
    )
    {
        dt = 0.05f;
    }


    // --------------------------------------------------------
    // PROPORTIONAL
    // --------------------------------------------------------

    float proportional =
        KP *
        error;


    // --------------------------------------------------------
    // INTEGRAL
    // --------------------------------------------------------

    integral +=
        error *
        dt;


    integral =
        constrain(
            integral,
            -PID_INTEGRAL_LIMIT,
            PID_INTEGRAL_LIMIT
        );


    float integralTerm =
        KI *
        integral;


    // --------------------------------------------------------
    // DERIVATIVE
    // --------------------------------------------------------

    float derivative =
        (
            error -
            previousError
        )
        /
        dt;


    float derivativeTerm =
        KD *
        derivative;


    // --------------------------------------------------------
    // SAVE ERROR
    // --------------------------------------------------------

    previousError =
        error;


    // --------------------------------------------------------
    // FINAL PID OUTPUT
    // --------------------------------------------------------

    float output =
        proportional +
        integralTerm +
        derivativeTerm;


    return output;
}


// ============================================================
// RESET PID
// ============================================================

void PIDController::reset()
{
    integral =
        0.0f;

    previousError =
        0.0f;

    previousTime =
        micros();
}


// ============================================================
// GET INTEGRAL
// ============================================================

float PIDController::getIntegral()
{
    return integral;
}


// ============================================================
// GET PREVIOUS ERROR
// ============================================================

float PIDController::getPreviousError()
{
    return previousError;
}