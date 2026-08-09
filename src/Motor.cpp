#include "Motor.h"

// ============================================================
// MOTOR MODULE
// TB6612FNG
// ============================================================
//
// Left motor:
// PWMA = GPIO18
// AIN1 = GPIO19
// AIN2 = GPIO20
//
// Right motor:
// PWMB = GPIO21
// BIN1 = GPIO22
// BIN2 = GPIO23
//
// STBY = GPIO7
// ============================================================


// ============================================================
// GLOBAL MOTOR OBJECT
// ============================================================

Motor motor;


// ============================================================
// BEGIN
// ============================================================

void Motor::begin()
{
    // --------------------------------------------------------
    // Direction pins
    // --------------------------------------------------------

    pinMode(
        AIN1,
        OUTPUT
    );

    pinMode(
        AIN2,
        OUTPUT
    );

    pinMode(
        BIN1,
        OUTPUT
    );

    pinMode(
        BIN2,
        OUTPUT
    );


    // --------------------------------------------------------
    // Standby
    // --------------------------------------------------------

    pinMode(
        STBY,
        OUTPUT
    );


    // --------------------------------------------------------
    // PWM
    // --------------------------------------------------------
    //
    // ESP32 Arduino Core 3.x
    // uses ledcAttach() rather than the old
    // ledcSetup()/ledcAttachPin() method.
    // --------------------------------------------------------

    bool leftPWMOK =
        ledcAttach(
            PWMA,
            PWM_FREQUENCY,
            PWM_RESOLUTION
        );


    bool rightPWMOK =
        ledcAttach(
            PWMB,
            PWM_FREQUENCY,
            PWM_RESOLUTION
        );


    if (
        !leftPWMOK ||
        !rightPWMOK
    )
    {
        Serial.println(
            "[MOTOR] PWM attach failed!"
        );
    }


    // --------------------------------------------------------
    // Initial safe state
    // --------------------------------------------------------

    digitalWrite(
        STBY,
        LOW
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


    ledcWrite(
        PWMA,
        0
    );

    ledcWrite(
        PWMB,
        0
    );


    leftPWMChannel =
        PWMA;

    rightPWMChannel =
        PWMB;


    Serial.println(
        "[MOTOR] Initialized"
    );
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


    Serial.println(
        "[MOTOR] ENABLED"
    );
}


// ============================================================
// DISABLE
// ============================================================

void Motor::disable()
{
    stop();


    digitalWrite(
        STBY,
        LOW
    );


    Serial.println(
        "[MOTOR] DISABLED"
    );
}


// ============================================================
// WRITE ONE MOTOR
// ============================================================
//
// speed:
//
// +255 = forward
//    0 = stop
// -255 = reverse
// ============================================================

void Motor::writeMotor(
    int in1,
    int in2,
    int pwmChannel,
    int speed,
    bool inverted
)
{
    // --------------------------------------------------------
    // Limit speed
    // --------------------------------------------------------

    speed =
        constrain(
            speed,
            -PWM_MAX,
            PWM_MAX
        );


    // --------------------------------------------------------
    // Apply motor inversion
    // --------------------------------------------------------

    if (
        inverted
    )
    {
        speed =
            -speed;
    }


    // --------------------------------------------------------
    // STOP
    // --------------------------------------------------------

    if (
        speed == 0
    )
    {
        digitalWrite(
            in1,
            LOW
        );

        digitalWrite(
            in2,
            LOW
        );

        ledcWrite(
            pwmChannel,
            0
        );

        return;
    }


    // --------------------------------------------------------
    // FORWARD
    // --------------------------------------------------------

    if (
        speed > 0
    )
    {
        digitalWrite(
            in1,
            HIGH
        );

        digitalWrite(
            in2,
            LOW
        );
    }


    // --------------------------------------------------------
    // REVERSE
    // --------------------------------------------------------

    else
    {
        digitalWrite(
            in1,
            LOW
        );

        digitalWrite(
            in2,
            HIGH
        );


        speed =
            -speed;
    }


    // --------------------------------------------------------
    // PWM
    // --------------------------------------------------------

    speed =
        constrain(
            speed,
            0,
            PWM_MAX
        );


    ledcWrite(
        pwmChannel,
        speed
    );
}


// ============================================================
// LEFT MOTOR
// ============================================================

void Motor::setLeft(
    int speed
)
{
    writeMotor(
        AIN1,
        AIN2,
        PWMA,
        speed,
        LEFT_MOTOR_INVERTED
    );
}


// ============================================================
// RIGHT MOTOR
// ============================================================

void Motor::setRight(
    int speed
)
{
    writeMotor(
        BIN1,
        BIN2,
        PWMB,
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
    setLeft(
        leftSpeed
    );


    setRight(
        rightSpeed
    );
}


// ============================================================
// NORMAL STOP
// ============================================================

void Motor::stop()
{
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


    ledcWrite(
        PWMA,
        0
    );

    ledcWrite(
        PWMB,
        0
    );
}


// ============================================================
// EMERGENCY STOP
// ============================================================

void Motor::emergencyStop()
{
    stop();


    digitalWrite(
        STBY,
        LOW
    );


    Serial.println(
        "[MOTOR] EMERGENCY STOP"
    );
}