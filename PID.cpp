#include "PID.h"

PID::PID(float p, float i, float d)
{
    kp = p;
    ki = i;
    kd = d;

    previousError = 0;
    integral = 0;
}

void PID::reset()
{
    previousError = 0;
    integral = 0;
}

int PID::compute(int error)
{
    integral = constrain(integral + error, -10000.0f, 10000.0f);

    float derivative = error - previousError;

    previousError = error;

    float output =
        kp * error +
        ki * integral +
        kd * derivative;

    return (int)output;
}