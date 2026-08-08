// ============================================================
// Motor.h  —  TB6612FNG dual-motor driver
// ============================================================
// Hardware:
//   TB6612FNG motor driver
//   PWMA -> GPIO 18   (Left motor PWM)
//   AIN1 -> GPIO 19
//   AIN2 -> GPIO 20
//   PWMB -> GPIO 21   (Right motor PWM)
//   BIN1 -> GPIO 22
//   BIN2 -> GPIO 23
//   STBY -> GPIO 7    (software standby control)
//
// STBY must be LOW during initialisation and calibration.
// STBY must be HIGH before any motor movement.
//
// Speed convention:
//   +255 = full forward
//   -255 = full reverse
//      0 = coast / stop
// ============================================================

#pragma once
#include <Arduino.h>
#include "Config.h"

class Motor
{
public:
    // ---- Lifecycle -------------------------------------------
    void begin();     // Initialise GPIO + PWM; STBY LOW; motors stopped

    // ---- Standby ---------------------------------------------
    void enable();    // STBY HIGH  — motor driver active
    void disable();   // STBY LOW   — motor driver in standby

    // ---- Movement control ------------------------------------
    // leftSpeed, rightSpeed: -255 to +255
    // Positive = forward, Negative = reverse, 0 = coast
    void setMotors(int leftSpeed, int rightSpeed);

    void setLeftMotor(int speed);
    void setRightMotor(int speed);

    // Coast to stop (PWM = 0, IN1=IN2=LOW)
    void stop();

    // Active brake (IN1=IN2=HIGH, PWM = 0)
    void brake();

    // ---- State query -----------------------------------------
    bool isEnabled() const { return _enabled; }

private:
    bool _enabled;

    // Drive one TB6612 channel
    // pwmPin:  PWMA or PWMB
    // in1,in2: direction GPIO pair
    // speed:   -255 to +255
    void driveChannel(uint8_t pwmPin, uint8_t in1, uint8_t in2, int speed);
};
