#ifndef ROBOT_H
#define ROBOT_H

#include "Sensor.h"
#include "Motor.h"
#include "PID.h"

class Robot
{
public:

    void begin();

    void calibrate();

    void followLine();

private:

    void searchLine();

    Sensor sensor;
    Motor motor;

    PID pid = PID(KP, KI, KD);

    int baseSpeed = BASE_SPEED;
};

#endif