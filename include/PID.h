#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID
{
private:
    float kp;
    float ki;
    float kd;

    float previousError;
    float integral;

public:

    PID(float p, float i, float d);

    void reset();

    int compute(int error);
};

#endif