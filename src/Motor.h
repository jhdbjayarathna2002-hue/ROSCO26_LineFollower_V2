#pragma once

#include <Arduino.h>
#include "Config.h"


class Motor
{
public:

    void begin();

    void enable();

    void disable();

    void setLeftMotor(int speed);

    void setRightMotor(int speed);

    void setMotors(
        int leftSpeed,
        int rightSpeed
    );

    void stop();

    void brake();

    bool isEnabled() const;


private:

    bool enabled = false;


    void writeMotor(
        uint8_t pwmPin,
        uint8_t in1Pin,
        uint8_t in2Pin,
        int speed,
        bool inverted
    );


    int limitSpeed(int speed);
};