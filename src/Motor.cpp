// ============================================================
// Motor.cpp  —  TB6612FNG dual-motor driver
// ============================================================
// See Motor.h for full documentation.
// ============================================================

#include "Motor.h"

// ============================================================
// begin()
// ============================================================

void Motor::begin()
{
    // Direction GPIO
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);

    // Standby GPIO
    pinMode(STBY, OUTPUT);

    // Configure PWM channels using Arduino-ESP32 Core 3.x API
    ledcAttach(PWMA, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttach(PWMB, PWM_FREQUENCY, PWM_RESOLUTION);

    _enabled = false;

    // Safety: disable driver and stop
    disable();
    stop();
}

// ============================================================
// enable() / disable()
// ============================================================

void Motor::enable()
{
    digitalWrite(STBY, HIGH);
    _enabled = true;

#ifdef DEBUG_MOTOR
    Serial.println(F("[MOTOR] STBY HIGH — driver enabled"));
#endif
}

void Motor::disable()
{
    stop();
    digitalWrite(STBY, LOW);
    _enabled = false;

#ifdef DEBUG_MOTOR
    Serial.println(F("[MOTOR] STBY LOW — driver disabled"));
#endif
}

// ============================================================
// stop()  —  coast (PWM = 0, IN1 = IN2 = LOW)
// ============================================================

void Motor::stop()
{
    ledcWrite(PWMA, 0);
    ledcWrite(PWMB, 0);

    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
}

// ============================================================
// brake()  —  active short-circuit brake (IN1 = IN2 = HIGH)
// ============================================================

void Motor::brake()
{
    ledcWrite(PWMA, 0);
    ledcWrite(PWMB, 0);

    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, HIGH);
}

// ============================================================
// Private: driveChannel()
// Sets direction and PWM for one TB6612 output channel.
// speed: -255 to +255 (clamped inside)
// ============================================================

void Motor::driveChannel(uint8_t pwmPin, uint8_t in1, uint8_t in2, int speed)
{
    speed = constrain(speed, -PWM_MAX, PWM_MAX);

    if (speed > 0)
    {
        // Forward
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        ledcWrite(pwmPin, (uint32_t)speed);
    }
    else if (speed < 0)
    {
        // Reverse
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        ledcWrite(pwmPin, (uint32_t)(-speed));
    }
    else
    {
        // Coast
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        ledcWrite(pwmPin, 0);
    }
}

// ============================================================
// setLeftMotor()
// Applies LEFT_MOTOR_INVERTED if configured.
// ============================================================

void Motor::setLeftMotor(int speed)
{
#if LEFT_MOTOR_INVERTED
    speed = -speed;
#endif
    driveChannel(PWMA, AIN1, AIN2, speed);
}

// ============================================================
// setRightMotor()
// Applies RIGHT_MOTOR_INVERTED if configured.
// ============================================================

void Motor::setRightMotor(int speed)
{
#if RIGHT_MOTOR_INVERTED
    speed = -speed;
#endif
    driveChannel(PWMB, BIN1, BIN2, speed);
}

// ============================================================
// setMotors()
// ============================================================

void Motor::setMotors(int leftSpeed, int rightSpeed)
{
    setLeftMotor(leftSpeed);
    setRightMotor(rightSpeed);

#ifdef DEBUG_MOTOR
    Serial.printf("[MOTOR] L=%d  R=%d\n", leftSpeed, rightSpeed);
#endif
}