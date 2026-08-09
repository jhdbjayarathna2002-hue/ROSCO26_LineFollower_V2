#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "Config.h"

class Motor
{
public:

    void begin();

    void setSpeed(int leftSpeed, int rightSpeed);

    void stop();

    void brake();

private:

    void driveMotor(uint8_t pwmPin,
                uint8_t in1,
                uint8_t in2,
                int speed);
};

#endif