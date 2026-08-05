#include "Motor.h"

//--------------------------------------------------
// Initialize Motors
//--------------------------------------------------

void Motor::begin()
{
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // Arduino ESP32 Core 3.x API
    ledcAttach(ENA, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttach(ENB, PWM_FREQUENCY, PWM_RESOLUTION);

    stop();
}

//--------------------------------------------------
// Set Left and Right Motor Speed
//--------------------------------------------------

void Motor::setSpeed(int leftSpeed, int rightSpeed)
{
    driveMotor(ENA, IN1, IN2, leftSpeed);
    driveMotor(ENB, IN3, IN4, rightSpeed);
}

//--------------------------------------------------
// Drive Single Motor
//--------------------------------------------------

void Motor::driveMotor(uint8_t pwmPin,
                       uint8_t in1,
                       uint8_t in2,
                       int speed)
{
    speed = constrain(speed, -255, 255);

    if (speed > 0)
    {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);

        ledcWrite(pwmPin, speed);
    }
    else if (speed < 0)
    {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);

        ledcWrite(pwmPin, -speed);
    }
    else
    {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);

        ledcWrite(pwmPin, 0);
    }
}

//--------------------------------------------------
// Stop
//--------------------------------------------------

void Motor::stop()
{
    ledcWrite(ENA, 0);
    ledcWrite(ENB, 0);

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

//--------------------------------------------------
// Brake
//--------------------------------------------------

void Motor::brake()
{
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, HIGH);

    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, HIGH);

    ledcWrite(ENA, 0);
    ledcWrite(ENB, 0);
}