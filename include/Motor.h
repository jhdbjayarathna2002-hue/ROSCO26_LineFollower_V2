#pragma once

#include <Arduino.h>
#include "Config.h"

// ============================================================
// MOTOR MODULE
// ============================================================
//
// TB6612FNG
//
// LEFT MOTOR:
// PWMA  -> GPIO18
// AIN1  -> GPIO19
// AIN2  -> GPIO20
//
// RIGHT MOTOR:
// PWMB  -> GPIO21
// BIN1  -> GPIO22
// BIN2  -> GPIO23
//
// STBY:
// GPIO7
//
// This module only controls the motors.
// Track detection and PID are handled elsewhere.
// ============================================================

class Motor
{
public:

    // --------------------------------------------------------
    // Initialize motor driver
    // --------------------------------------------------------

    void begin();


    // --------------------------------------------------------
    // Enable motor driver
    // --------------------------------------------------------

    void enable();


    // --------------------------------------------------------
    // Disable motor driver
    // --------------------------------------------------------

    void disable();


    // --------------------------------------------------------
    // Set individual motor speeds
    //
    // Range:
    //
    // -255 = full reverse
    //    0 = stop
    // +255 = full forward
    // --------------------------------------------------------

    void setLeft(
        int speed
    );


    void setRight(
        int speed
    );


    // --------------------------------------------------------
    // Set both motors
    // --------------------------------------------------------

    void setMotors(
        int leftSpeed,
        int rightSpeed
    );


    // --------------------------------------------------------
    // Stop both motors
    // --------------------------------------------------------

    void stop();


    // --------------------------------------------------------
    // Emergency stop
    // --------------------------------------------------------

    void emergencyStop();


private:

    // --------------------------------------------------------
    // PWM channels
    // --------------------------------------------------------

    int leftPWMChannel;

    int rightPWMChannel;


    // --------------------------------------------------------
    // Internal motor writer
    // --------------------------------------------------------

    void writeMotor(
        int in1,
        int in2,
        int pwmChannel,
        int speed,
        bool inverted
    );
};


// ============================================================
// GLOBAL MOTOR OBJECT
// ============================================================

extern Motor motor;