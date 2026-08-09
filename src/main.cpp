#include <Arduino.h>
#include "Config.h"
#include "Motor.h"

// ============================================================
// ROSCO'26 TASK 01
// BASIC COMPLETE LINE FOLLOWER
//
// TRACK:
// BLACK LINE
// WHITE BACKGROUND
//
// CURRENT FEATURES:
// 1. 16-channel IR sensor reading
// 2. Individual white calibration
// 3. Individual black calibration
// 4. Individual sensor thresholds
// 5. Calibration only once at startup
// 6. Black-line inversion
// 7. Line position calculation
// 8. Basic PID
// 9. Basic line following
// 10. Basic line-loss recovery
//
// NOT YET:
// - 90 degree special turn
// - T/L junction strategy
// - intersection strategy
// - circle strategy
// - dead-end strategy
// ============================================================


// ============================================================
// MOTOR
// ============================================================

Motor motor;


// ============================================================
// CALIBRATION DATA
// ============================================================

int whiteValue[SENSOR_COUNT];

int blackValue[SENSOR_COUNT];

int thresholdValue[SENSOR_COUNT];

bool sensorValid[SENSOR_COUNT];


// ============================================================
// SENSOR DATA
// ============================================================

int rawValue[SENSOR_COUNT];

int lineStrength[SENSOR_COUNT];


// ============================================================
// SENSOR POSITION WEIGHTS
// ============================================================
//
// C0  = far left
// C7  = left of center
// C8  = right of center
// C15 = far right
//
// Center is approximately between C7 and C8.
// ============================================================

const int sensorWeight[SENSOR_COUNT] =
{
    -7500,
    -6500,
    -5500,
    -4500,
    -3500,
    -2500,
    -1500,
     -500,
     +500,
    +1500,
    +2500,
    +3500,
    +4500,
    +5500,
    +6500,
    +7500
};


// ============================================================
// PID VARIABLES
// ============================================================

float pidIntegral = 0.0f;

float previousError = 0.0f;

unsigned long previousPIDTime = 0;


// ============================================================
// LINE POSITION
// ============================================================

int linePosition = 0;

int previousPosition = 0;

bool lineDetected = false;

int activeSensorCount = 0;

long totalLineStrength = 0;


// ============================================================
// LINE LOSS
// ============================================================

unsigned long lineLostStart = 0;

unsigned long searchStart = 0;


// ============================================================
// CALIBRATION STATUS
// ============================================================

bool calibrationFinished = false;


// ============================================================
// RGB LED
// ============================================================

void ledOff()
{
    rgbLedWrite(
        RGB_LED_PIN,
        0,
        0,
        0
    );
}


void ledBlue()
{
    rgbLedWrite(
        RGB_LED_PIN,
        0,
        0,
        RGB_LED_BRIGHTNESS
    );
}


void ledGreen()
{
    rgbLedWrite(
        RGB_LED_PIN,
        0,
        RGB_LED_BRIGHTNESS,
        0
    );
}


void ledRed()
{
    rgbLedWrite(
        RGB_LED_PIN,
        RGB_LED_BRIGHTNESS,
        0,
        0
    );
}


// ============================================================
// MUX CHANNEL SELECTION
// ============================================================

void selectChannel(uint8_t channel)
{
    digitalWrite(
        MUX_S0,
        channel & 0x01
    );

    digitalWrite(
        MUX_S1,
        (channel >> 1) & 0x01
    );

    digitalWrite(
        MUX_S2,
        (channel >> 2) & 0x01
    );

    digitalWrite(
        MUX_S3,
        (channel >> 3) & 0x01
    );

    delayMicroseconds(
        MUX_SETTLE_US
    );
}


// ============================================================
// READ ONE SENSOR
// ============================================================

int readSensor(uint8_t channel)
{
    selectChannel(channel);

    long total = 0;

    for (
        int i = 0;
        i < SENSOR_AVERAGE_SAMPLES;
        i++
    )
    {
        total += analogRead(MUX_SIG);

        delayMicroseconds(30);
    }

    return total / SENSOR_AVERAGE_SAMPLES;
}


// ============================================================
// READ ALL 16 SENSORS
// ============================================================

void readAllSensors()
{
    for (
        int i = 0;
        i < SENSOR_COUNT;
        i++
    )
    {
        rawValue[i] = readSensor(i);
    }
}


// ============================================================
// CALIBRATION COUNTDOWN
// ============================================================

void countdown()
{
    for (
        int i = 3;
        i >= 1;
        i--
    )
    {
        Serial.print(i);
        Serial.println("...");

        delay(1000);
    }
}


// ============================================================
// WHITE CALIBRATION
// ============================================================
//
// HIGH ADC = WHITE
// ============================================================

void calibrateWhite()
{
    Serial.println();
    Serial.println(
        "========================================"
    );

    Serial.println(
        "WHITE CALIBRATION"
    );

    Serial.println(
        "========================================"
    );

    Serial.println();

    Serial.println(
        "Place ALL 16 sensors over WHITE."
    );

    Serial.println(
        "BLUE LED = WHITE CALIBRATION"
    );

    ledBlue();

    countdown();

    Serial.println();
    Serial.println(
        "Measuring WHITE..."
    );

    for (
        int sensor = 0;
        sensor < SENSOR_COUNT;
        sensor++
    )
    {
        long total = 0;

        for (
            int sample = 0;
            sample < CALIBRATION_SAMPLES;
            sample++
        )
        {
            total += readSensor(sensor);

            delay(
                CALIBRATION_SAMPLE_DELAY_MS
            );
        }

        whiteValue[sensor] =
            total / CALIBRATION_SAMPLES;

        Serial.print("C");
        Serial.print(sensor);
        Serial.print(" WHITE = ");
        Serial.println(
            whiteValue[sensor]
        );
    }

    ledOff();

    Serial.println();
    Serial.println(
        "WHITE CALIBRATION COMPLETE."
    );
}


// ============================================================
// BLACK CALIBRATION
// ============================================================
//
// LOW ADC = BLACK
// ============================================================

void calibrateBlack()
{
    Serial.println();
    Serial.println(
        "========================================"
    );

    Serial.println(
        "BLACK CALIBRATION"
    );

    Serial.println(
        "========================================"
    );

    Serial.println();

    Serial.println(
        "Place ALL 16 sensors over BLACK LINE."
    );

    Serial.println(
        "GREEN LED = BLACK CALIBRATION"
    );

    ledGreen();

    countdown();

    Serial.println();
    Serial.println(
        "Measuring BLACK..."
    );

    for (
        int sensor = 0;
        sensor < SENSOR_COUNT;
        sensor++
    )
    {
        long total = 0;

        for (
            int sample = 0;
            sample < CALIBRATION_SAMPLES;
            sample++
        )
        {
            total += readSensor(sensor);

            delay(
                CALIBRATION_SAMPLE_DELAY_MS
            );
        }

        blackValue[sensor] =
            total / CALIBRATION_SAMPLES;

        Serial.print("C");
        Serial.print(sensor);
        Serial.print(" BLACK = ");
        Serial.println(
            blackValue[sensor]
        );
    }

    ledOff();

    Serial.println();
    Serial.println(
        "BLACK CALIBRATION COMPLETE."
    );
}


// ============================================================
// CREATE INDIVIDUAL CALIBRATION
// ============================================================
//
// Every sensor has its own:
//
// C0  → black + white + threshold
// C1  → black + white + threshold
// ...
// C15 → black + white + threshold
//
// NO COMMON ADC THRESHOLD.
// ============================================================

bool createCalibration()
{
    bool allValid = true;

    Serial.println();
    Serial.println(
        "========================================"
    );

    Serial.println(
        "INDIVIDUAL SENSOR CALIBRATION"
    );

    Serial.println(
        "========================================"
    );

    for (
        int i = 0;
        i < SENSOR_COUNT;
        i++
    )
    {
        int range =
            whiteValue[i] -
            blackValue[i];

        thresholdValue[i] =
            blackValue[i] +
            range / 2;

        if (
            range >= MIN_CALIBRATION_RANGE
        )
        {
            sensorValid[i] = true;
        }
        else
        {
            sensorValid[i] = false;

            allValid = false;
        }

        Serial.print("C");
        Serial.print(i);

        Serial.print(
            " | BLACK="
        );

        Serial.print(
            blackValue[i]
        );

        Serial.print(
            " | WHITE="
        );

        Serial.print(
            whiteValue[i]
        );

        Serial.print(
            " | THRESHOLD="
        );

        Serial.print(
            thresholdValue[i]
        );

        Serial.print(
            " | RANGE="
        );

        Serial.print(range);

        if (sensorValid[i])
        {
            Serial.println(
                " | VALID"
            );
        }
        else
        {
            Serial.println(
                " | INVALID"
            );
        }
    }

    return allValid;
}


// ============================================================
// NORMALIZE ONE SENSOR
// ============================================================
//
// First:
// BLACK = 0
// WHITE = 1000
//
// Then because our track is:
//
// BLACK LINE
// WHITE BACKGROUND
//
// invert:
//
// BLACK LINE = 1000
// WHITE       = 0
// ============================================================

int normalizeSensor(int sensor)
{
    if (!sensorValid[sensor])
    {
        return 0;
    }

    int black =
        blackValue[sensor];

    int white =
        whiteValue[sensor];

    int raw =
        rawValue[sensor];

    int range =
        white - black;

    if (range <= 0)
    {
        return 0;
    }

    long value =
        (
            (long)(raw - black)
            * 1000L
        ) / range;

    value = constrain(
        value,
        0,
        1000
    );

    // ========================================================
    // BLACK LINE / WHITE BACKGROUND
    // ========================================================

    if (INVERT_LINE_FOLLOWING)
    {
        value = 1000 - value;
    }

    return (int)value;
}


// ============================================================
// PROCESS SENSOR ARRAY
// ============================================================

void processSensors()
{
    activeSensorCount = 0;

    totalLineStrength = 0;

    long weightedSum = 0;

    for (
        int i = 0;
        i < SENSOR_COUNT;
        i++
    )
    {
        lineStrength[i] =
            normalizeSensor(i);

        // Individual threshold only.
        // Each sensor uses its OWN threshold.

        if (
            rawValue[i] <=
            thresholdValue[i]
        )
        {
            activeSensorCount++;
        }

        int strength =
            lineStrength[i];

        if (strength < 100)
        {
            strength = 0;
        }

        weightedSum +=
            (long)sensorWeight[i] *
            strength;

        totalLineStrength +=
            strength;
    }

    // --------------------------------------------------------
    // LINE DETECTION
    // --------------------------------------------------------

    lineDetected =
        (
            totalLineStrength >=
            MIN_LINE_STRENGTH_SUM
        );

    // --------------------------------------------------------
    // LINE POSITION
    // --------------------------------------------------------

    if (totalLineStrength > 0)
    {
        linePosition =
            weightedSum /
            totalLineStrength;

        previousPosition =
            linePosition;
    }
}


// ============================================================
// PID
// ============================================================

float calculatePID(float error)
{
    unsigned long now = micros();

    float dt;

    if (previousPIDTime == 0)
    {
        dt = 0.002f;
    }
    else
    {
        dt =
            (
                now -
                previousPIDTime
            ) / 1000000.0f;
    }

    previousPIDTime = now;

    if (dt <= 0.0f)
    {
        dt = 0.002f;
    }

    if (dt > 0.05f)
    {
        dt = 0.05f;
    }

    // --------------------------------------------------------
    // Integral
    // --------------------------------------------------------

    pidIntegral +=
        error * dt;

    pidIntegral =
        constrain(
            pidIntegral,
            -PID_INTEGRAL_LIMIT,
            PID_INTEGRAL_LIMIT
        );

    // --------------------------------------------------------
    // Derivative
    // --------------------------------------------------------

    float derivative =
        (
            error -
            previousError
        ) / dt;

    // --------------------------------------------------------
    // PID OUTPUT
    // --------------------------------------------------------

    float output =
        (KP * error) +
        (KI * pidIntegral) +
        (KD * derivative);

    previousError =
        error;

    return output;
}


// ============================================================
// MINIMUM MOTOR PWM
// ============================================================

int applyMinimumPWM(int speed)
{
    if (speed == 0)
    {
        return 0;
    }

    if (
        speed > 0 &&
        speed < MOTOR_MIN_PWM
    )
    {
        return MOTOR_MIN_PWM;
    }

    return speed;
}


// ============================================================
// DRIVE MOTORS
// ============================================================

void drive(
    int left,
    int right
)
{
    left =
        constrain(
            left,
            -MAX_SPEED,
            MAX_SPEED
        );

    right =
        constrain(
            right,
            -MAX_SPEED,
            MAX_SPEED
        );

    motor.setMotors(
        left,
        right
    );
}


// ============================================================
// BASIC LINE FOLLOWING
// ============================================================
//
// BLACK LINE:
//     left sensor activation  → steer left
//     right sensor activation → steer right
//
// This is the corrected steering relationship.
// ============================================================

void followLine()
{
    float error =
        linePosition;

    float correction =
        calculatePID(error);

    int baseSpeed =
        BASE_SPEED;

    int left =
        baseSpeed +
        (int)correction;

    int right =
        baseSpeed -
        (int)correction;

    left =
        constrain(
            left,
            0,
            MAX_SPEED
        );

    right =
        constrain(
            right,
            0,
            MAX_SPEED
        );

    left =
        applyMinimumPWM(left);

    right =
        applyMinimumPWM(right);

    drive(
        left,
        right
    );
}


// ============================================================
// LINE LOSS RECOVERY
// ============================================================
//
// Short loss:
//     continue toward previous line position
//
// Longer loss:
//     search for the line
// ============================================================

void searchForLine()
{
    unsigned long now =
        millis();

    if (lineLostStart == 0)
    {
        lineLostStart =
            now;

        searchStart =
            now;

        pidIntegral = 0;

        Serial.println(
            "[LINE] LOST"
        );
    }

    unsigned long lostTime =
        now -
        lineLostStart;

    // --------------------------------------------------------
    // SHORT LOSS
    // --------------------------------------------------------

    if (
        lostTime <
        DASHED_LINE_TIME_MS
    )
    {
        if (previousPosition < 0)
        {
            drive(
                BASE_SPEED - 15,
                BASE_SPEED + 15
            );
        }
        else
        {
            drive(
                BASE_SPEED + 15,
                BASE_SPEED - 15
            );
        }

        return;
    }

    // --------------------------------------------------------
    // SEARCH
    // --------------------------------------------------------

    if (
        lostTime <
        SEARCH_TIMEOUT_MS
    )
    {
        if (previousPosition < 0)
        {
            drive(
                -SEARCH_SPEED,
                SEARCH_SPEED
            );
        }
        else
        {
            drive(
                SEARCH_SPEED,
                -SEARCH_SPEED
            );
        }

        return;
    }

    // --------------------------------------------------------
    // SEARCH TIMEOUT
    // --------------------------------------------------------

    motor.stop();

    Serial.println(
        "[LINE] SEARCH TIMEOUT"
    );

    // Don't permanently lock the robot.
    // Give it a short pause and allow another search.

    delay(100);

    lineLostStart = 0;
}


// ============================================================
// DEBUG OUTPUT
// ============================================================

void printDebug()
{
    Serial.print(
        "POS="
    );

    Serial.print(
        linePosition
    );

    Serial.print(
        "  ACTIVE="
    );

    Serial.print(
        activeSensorCount
    );

    Serial.print(
        "  SUM="
    );

    Serial.print(
        totalLineStrength
    );

    Serial.print(
        "  RAW="
    );

    for (
        int i = 0;
        i < SENSOR_COUNT;
        i++
    )
    {
        Serial.print(
            rawValue[i]
        );

        if (
            i <
            SENSOR_COUNT - 1
        )
        {
            Serial.print(",");
        }
    }

    Serial.println();
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(
        SERIAL_BAUD
    );

    delay(1500);

    Serial.println();
    Serial.println();

    Serial.println(
        "########################################"
    );

    Serial.println(
        "# ROSCO'26 TASK 01"
    );

    Serial.println(
        "# BLACK LINE / WHITE BACKGROUND"
    );

    Serial.println(
        "########################################"
    );

    // ========================================================
    // MUX PINS
    // ========================================================

    pinMode(
        MUX_S0,
        OUTPUT
    );

    pinMode(
        MUX_S1,
        OUTPUT
    );

    pinMode(
        MUX_S2,
        OUTPUT
    );

    pinMode(
        MUX_S3,
        OUTPUT
    );

    pinMode(
        MUX_SIG,
        INPUT
    );

    // ========================================================
    // ADC
    // ========================================================

    analogReadResolution(
        ADC_RESOLUTION
    );

    // ========================================================
    // INITIAL MUX STATE
    // ========================================================

    digitalWrite(
        MUX_S0,
        LOW
    );

    digitalWrite(
        MUX_S1,
        LOW
    );

    digitalWrite(
        MUX_S2,
        LOW
    );

    digitalWrite(
        MUX_S3,
        LOW
    );

    // ========================================================
    // LED
    // ========================================================

    ledOff();

    // ========================================================
    // MOTOR
    // ========================================================

    motor.begin();

    // Motors remain disabled during calibration.

    motor.disable();

    // ========================================================
    // CALIBRATION
    // ========================================================

    Serial.println();

    Serial.println(
        "CALIBRATION STARTED"
    );

    Serial.println(
        "MOTORS DISABLED"
    );

    // --------------------------------------------------------
    // WHITE
    // --------------------------------------------------------

    calibrateWhite();

    delay(500);

    // --------------------------------------------------------
    // BLACK
    // --------------------------------------------------------

    calibrateBlack();

    delay(500);

    // --------------------------------------------------------
    // BUILD INDIVIDUAL CALIBRATION
    // --------------------------------------------------------

    bool calibrationOK =
        createCalibration();

    // ========================================================
    // CALIBRATION FAILURE
    // ========================================================

    if (!calibrationOK)
    {
        Serial.println();

        Serial.println(
            "################################"
        );

        Serial.println(
            "CALIBRATION FAILED"
        );

        Serial.println(
            "CHECK SENSOR VALUES"
        );

        Serial.println(
            "################################"
        );

        ledRed();

        while (true)
        {
            motor.disable();

            delay(1000);
        }
    }

    // ========================================================
    // CALIBRATION SUCCESS
    // ========================================================

    calibrationFinished =
        true;

    Serial.println();

    Serial.println(
        "################################"
    );

    Serial.println(
        "CALIBRATION SUCCESSFUL"
    );

    Serial.println(
        "ALL 16 SENSORS VALID"
    );

    Serial.println(
        "CALIBRATION LOCKED"
    );

    Serial.println(
        "################################"
    );

    // Green = ready

    ledGreen();

    delay(1500);

    ledOff();

    // ========================================================
    // RESET PID
    // ========================================================

    pidIntegral = 0.0f;

    previousError = 0.0f;

    previousPosition = 0;

    previousPIDTime = micros();

    // ========================================================
    // ENABLE MOTORS
    // ========================================================

    motor.enable();

    Serial.println();

    Serial.println(
        "========================================"
    );

    Serial.println(
        "TASK 01 LINE FOLLOWER READY"
    );

    Serial.println(
        "BLACK LINE MODE = ON"
    );

    Serial.println(
        "========================================"
    );
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
    // --------------------------------------------------------
    // Safety
    // --------------------------------------------------------

    if (!calibrationFinished)
    {
        motor.stop();

        return;
    }

    // --------------------------------------------------------
    // Read sensors at fixed interval
    // --------------------------------------------------------

    static unsigned long lastLoopTime = 0;

    unsigned long now =
        micros();

    if (
        now -
        lastLoopTime <
        CONTROL_LOOP_US
    )
    {
        return;
    }

    lastLoopTime =
        now;

    // --------------------------------------------------------
    // READ 16 SENSORS
    // --------------------------------------------------------

    readAllSensors();

    // --------------------------------------------------------
    // NORMALIZE + POSITION
    // --------------------------------------------------------

    processSensors();

    // --------------------------------------------------------
    // LINE LOST
    // --------------------------------------------------------

    if (!lineDetected)
    {
        searchForLine();

        return;
    }

    // --------------------------------------------------------
    // LINE FOUND
    // --------------------------------------------------------

    lineLostStart = 0;

    searchStart = 0;

    // --------------------------------------------------------
    // BASIC PID LINE FOLLOWING
    // --------------------------------------------------------

    followLine();

    // --------------------------------------------------------
    // DEBUG
    // --------------------------------------------------------

#ifdef DEBUG_SENSOR

    printDebug();

#endif
}