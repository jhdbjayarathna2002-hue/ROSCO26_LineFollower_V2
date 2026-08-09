#include "Motor.h"


// ============================================================
// MOTOR INITIALIZATION
// ============================================================

void Motor::begin()
{
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);

    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);

    pinMode(STBY, OUTPUT);

    // Driver disabled at startup
    digitalWrite(STBY, LOW);

    enabled = false;


    // ESP32 Arduino Core 3.x PWM

    ledcAttach(
        PWMA,
        PWM_FREQUENCY,
        PWM_RESOLUTION
    );

    ledcAttach(
        PWMB,
        PWM_FREQUENCY,
        PWM_RESOLUTION
    );


    // PWM off

    ledcWrite(PWMA, 0);

    ledcWrite(PWMB, 0);


    // Motor outputs LOW

    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);

    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
}


// ============================================================
// ENABLE
// ============================================================

void Motor::enable()
{
    digitalWrite(
        STBY,
        HIGH
    );

    enabled = true;

    Serial.println(
        "[MOTOR] ENABLED"
    );
}


// ============================================================
// DISABLE
// ============================================================

void Motor::disable()
{
    ledcWrite(
        PWMA,
        0
    );

    ledcWrite(
        PWMB,
        0
    );


    digitalWrite(
        AIN1,
        LOW
    );

    digitalWrite(
        AIN2,
        LOW
    );


    digitalWrite(
        BIN1,
        LOW
    );

    digitalWrite(
        BIN2,
        LOW
    );


    digitalWrite(
        STBY,
        LOW
    );

    enabled = false;

    Serial.println(
        "[MOTOR] DISABLED"
    );
}


// ============================================================
// STATUS
// ============================================================

bool Motor::isEnabled() const
{
    return enabled;
}


// ============================================================
// LIMIT SPEED
// ============================================================

int Motor::limitSpeed(int speed)
{
    return constrain(
        speed,
        -PWM_MAX,
        PWM_MAX
    );
}


// ============================================================
// WRITE MOTOR
// ============================================================

void Motor::writeMotor(
    uint8_t pwmPin,
    uint8_t in1Pin,
    uint8_t in2Pin,
    int speed,
    bool inverted
)
{
    speed =
        limitSpeed(speed);


    if (inverted)
    {
        speed =
            -speed;
    }


    // --------------------------------------------------------
    // STOP / COAST
    // --------------------------------------------------------

    if (speed == 0)
    {
        ledcWrite(
            pwmPin,
            0
        );

        digitalWrite(
            in1Pin,
            LOW
        );

        digitalWrite(
            in2Pin,
            LOW
        );

        return;
    }


    // --------------------------------------------------------
    // FORWARD
    // --------------------------------------------------------

    if (speed > 0)
    {
        digitalWrite(
            in1Pin,
            HIGH
        );

        digitalWrite(
            in2Pin,
            LOW
        );

        ledcWrite(
            pwmPin,
            speed
        );

        return;
    }


    // --------------------------------------------------------
    // REVERSE
    // --------------------------------------------------------

    digitalWrite(
        in1Pin,
        LOW
    );

    digitalWrite(
        in2Pin,
        HIGH
    );

    ledcWrite(
        pwmPin,
        -speed
    );
}


// ============================================================
// LEFT MOTOR
// ============================================================

void Motor::setLeftMotor(
    int speed
)
{
    if (!enabled)
        return;


    writeMotor(
        PWMA,
        AIN1,
        AIN2,
        speed,
        LEFT_MOTOR_INVERTED
    );
}


// ============================================================
// RIGHT MOTOR
// ============================================================

void Motor::setRightMotor(
    int speed
)
{
    if (!enabled)
        return;


    writeMotor(
        PWMB,
        BIN1,
        BIN2,
        speed,
        RIGHT_MOTOR_INVERTED
    );
}


// ============================================================
// BOTH MOTORS
// ============================================================

void Motor::setMotors(
    int leftSpeed,
    int rightSpeed
)
{
    if (!enabled)
        return;


    setLeftMotor(
        leftSpeed
    );

    setRightMotor(
        rightSpeed
    );
}


// ============================================================
// NORMAL STOP
// ============================================================
//
// COAST.
// We do NOT actively brake during normal line following.
// ============================================================

void Motor::stop()
{
    ledcWrite(
        PWMA,
        0
    );

    ledcWrite(
        PWMB,
        0
    );


    digitalWrite(
        AIN1,
        LOW
    );

    digitalWrite(
        AIN2,
        LOW
    );


    digitalWrite(
        BIN1,
        LOW
    );

    digitalWrite(
        BIN2,
        LOW
    );
}


// ============================================================
// ACTIVE BRAKE
// ============================================================
//
// Not used for normal line following.
// ============================================================

void Motor::brake()
{
    ledcWrite(
        PWMA,
        PWM_MAX
    );

    ledcWrite(
        PWMB,
        PWM_MAX
    );


    digitalWrite(
        AIN1,
        HIGH
    );

    digitalWrite(
        AIN2,
        HIGH
    );


    digitalWrite(
        BIN1,
        HIGH
    );

    digitalWrite(
        BIN2,
        HIGH
    );
}