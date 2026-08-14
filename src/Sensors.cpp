#include "Sensors.h"

// ============================================================
// SENSOR POSITION WEIGHTS
// ============================================================
//
// C0  = far left
// C7  = center-left
// C8  = center-right
// C15 = far right
//
// C7 + C8 together represent the center.
// ============================================================

static const int sensorWeight[SENSOR_COUNT] =
{
    -7500,  // C0
    -6500,  // C1
    -5500,  // C2
    -4500,  // C3
    -3500,  // C4
    -2500,  // C5
    -1500,  // C6
     -500,  // C7
     +500,  // C8
    +1500,  // C9
    +2500,  // C10
    +3500,  // C11
    +4500,  // C12
    +5500,  // C13
    +6500,  // C14
    +7500   // C15
};


// ============================================================
// GLOBAL SENSOR OBJECT
// ============================================================

Sensors sensors;


// ============================================================
// BEGIN
// ============================================================

void Sensors::begin()
{
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


    analogReadResolution(
        ADC_RESOLUTION
    );


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


    calibrated = false;

    linePositionValue = 0;

    activeSensorCount = 0;

    totalLineStrengthValue = 0;

    lineDetectedValue = false;


    for (
        int i = 0;
        i < SENSOR_COUNT;
        i++
    )
    {
        rawValue[i] = 0;

        whiteValue[i] = 0;

        blackValue[i] = 0;

        thresholdValue[i] = 0;

        lineStrengthValue[i] = 0;

        sensorValid[i] = false;
    }
}


// ============================================================
// SELECT MUX CHANNEL
// ============================================================

void Sensors::selectChannel(
    uint8_t channel
)
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

int Sensors::readChannel(
    uint8_t channel
)
{
    selectChannel(
        channel
    );


    long total = 0;


    for (
        int sample = 0;
        sample < SENSOR_AVERAGE_SAMPLES;
        sample++
    )
    {
        total +=
            analogRead(
                MUX_SIG
            );


        delayMicroseconds(25);
    }


    return
        total /
        SENSOR_AVERAGE_SAMPLES;
}


// ============================================================
// WHITE CALIBRATION
// ============================================================
//
// Each sensor is measured independently.
// No common threshold is used.
// ============================================================

void Sensors::calibrateWhite()
{
    Serial.println();
    Serial.println(
        "########################################"
    );

    Serial.println(
        "WHITE CALIBRATION"
    );

    Serial.println(
        "########################################"
    );

    Serial.println();

    Serial.println(
        "Place ALL 16 sensors over WHITE."
    );

    Serial.println(
        "BLUE LED = WHITE CALIBRATION"
    );


    // --------------------------------------------------------
    // LED
    // --------------------------------------------------------

    rgbLedWrite(
        RGB_LED_PIN,
        0,
        0,
        RGB_LED_BRIGHTNESS
    );


    Serial.println(
        "Calibration will start in 3 seconds..."
    );


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


    Serial.println();

    Serial.println(
        "Measuring WHITE..."
    );


    // --------------------------------------------------------
    // Read C0 → C15
    // --------------------------------------------------------

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
            total +=
                readChannel(
                    sensor
                );


            delay(
                CALIBRATION_SAMPLE_DELAY_MS
            );
        }


        whiteValue[sensor] =
            total /
            CALIBRATION_SAMPLES;


        Serial.print("C");
        Serial.print(sensor);
        Serial.print(" WHITE = ");
        Serial.println(
            whiteValue[sensor]
        );
    }


    rgbLedWrite(
        RGB_LED_PIN,
        0,
        0,
        0
    );


    Serial.println();

    Serial.println(
        "WHITE CALIBRATION COMPLETE."
    );
}


// ============================================================
// BLACK CALIBRATION
// ============================================================

void Sensors::calibrateBlack()
{
    Serial.println();
    Serial.println(
        "########################################"
    );

    Serial.println(
        "BLACK CALIBRATION"
    );

    Serial.println(
        "########################################"
    );

    Serial.println();

    Serial.println(
        "Place ALL 16 sensors over BLACK."
    );

    Serial.println(
        "GREEN LED = BLACK CALIBRATION"
    );


    // --------------------------------------------------------
    // LED
    // --------------------------------------------------------

    rgbLedWrite(
        RGB_LED_PIN,
        0,
        RGB_LED_BRIGHTNESS,
        0
    );


    Serial.println(
        "Calibration will start in 3 seconds..."
    );


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


    Serial.println();

    Serial.println(
        "Measuring BLACK..."
    );


    // --------------------------------------------------------
    // Read C0 → C15
    // --------------------------------------------------------

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
            total +=
                readChannel(
                    sensor
                );


            delay(
                CALIBRATION_SAMPLE_DELAY_MS
            );
        }


        blackValue[sensor] =
            total /
            CALIBRATION_SAMPLES;


        Serial.print("C");
        Serial.print(sensor);
        Serial.print(" BLACK = ");
        Serial.println(
            blackValue[sensor]
        );
    }


    rgbLedWrite(
        RGB_LED_PIN,
        0,
        0,
        0
    );


    Serial.println();

    Serial.println(
        "BLACK CALIBRATION COMPLETE."
    );
}


// ============================================================
// CREATE INDIVIDUAL CALIBRATION
// ============================================================
//
// Every sensor gets:
//
// C0  → own black / white / threshold
// C1  → own black / white / threshold
// ...
// C15 → own black / white / threshold
//
// No common threshold.
// ============================================================

bool Sensors::createCalibration()
{
    bool allValid = true;


    Serial.println();

    Serial.println(
        "########################################"
    );

    Serial.println(
        "INDIVIDUAL SENSOR CALIBRATION"
    );

    Serial.println(
        "########################################"
    );


    for (
        int sensor = 0;
        sensor < SENSOR_COUNT;
        sensor++
    )
    {
        int range =
            whiteValue[sensor]
            -
            blackValue[sensor];


        thresholdValue[sensor] =
            blackValue[sensor]
            +
            (
                range / 2
            );


        if (
            range >=
            MIN_CALIBRATION_RANGE
        )
        {
            sensorValid[sensor] =
                true;
        }
        else
        {
            sensorValid[sensor] =
                false;

            allValid =
                false;
        }


        Serial.print("C");
        Serial.print(sensor);

        Serial.print(
            " | BLACK="
        );

        Serial.print(
            blackValue[sensor]
        );

        Serial.print(
            " | WHITE="
        );

        Serial.print(
            whiteValue[sensor]
        );

        Serial.print(
            " | THRESHOLD="
        );

        Serial.print(
            thresholdValue[sensor]
        );

        Serial.print(
            " | RANGE="
        );

        Serial.print(
            range
        );


        if (
            sensorValid[sensor]
        )
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
// COMPLETE CALIBRATION
// ============================================================
//
// This function is called ONCE from setup().
//
// Sequence:
//
// WHITE
//   ↓
// BLACK
//   ↓
// INDIVIDUAL VALUES
//   ↓
// LOCK
// ============================================================

bool Sensors::calibrate()
{
    if (calibrated)
    {
        Serial.println(
            "Calibration already locked."
        );

        return true;
    }


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
    // INDIVIDUAL CALIBRATION
    // --------------------------------------------------------

    bool valid =
        createCalibration();


    if (!valid)
    {
        Serial.println();

        Serial.println(
            "########################################"
        );

        Serial.println(
            "CALIBRATION FAILED"
        );

        Serial.println(
            "CHECK SENSOR VALUES"
        );

        Serial.println(
            "########################################"
        );


        rgbLedWrite(
            RGB_LED_PIN,
            RGB_LED_BRIGHTNESS,
            0,
            0
        );


        calibrated = false;

        return false;
    }


    // --------------------------------------------------------
    // LOCK
    // --------------------------------------------------------

    calibrated = true;


    Serial.println();

    Serial.println(
        "########################################"
    );

    Serial.println(
        "CALIBRATION LOCKED."
    );

    Serial.println(
        "NO FURTHER CALIBRATION WILL OCCUR."
    );

    Serial.println(
        "########################################"
    );


    // Green = ready

    rgbLedWrite(
        RGB_LED_PIN,
        0,
        RGB_LED_BRIGHTNESS,
        0
    );


    delay(1000);


    rgbLedWrite(
        RGB_LED_PIN,
        0,
        0,
        0
    );


    return true;
}


// ============================================================
// NORMALIZE SENSOR
// ============================================================
//
// First normalize:
//
// BLACK = 0
// WHITE = 1000
//
// Then apply track polarity.
//
// Current:
//
// BLACK LINE / WHITE BACKGROUND
//
// gives:
//
// BLACK LINE = 1000
// WHITE       = 0
//
// For official white-line mode:
//
// WHITE LINE = 1000
// BLACK       = 0
// ============================================================

int Sensors::normalize(
    uint8_t sensor
)
{
    if (
        sensor >=
        SENSOR_COUNT
    )
    {
        return 0;
    }


    if (
        !sensorValid[sensor]
    )
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
        white -
        black;


    if (
        range <= 0
    )
    {
        return 0;
    }


    long value =
        (
            (long)
            (
                raw -
                black
            )
            *
            1000L
        )
        /
        range;


    value =
        constrain(
            value,
            0,
            1000
        );


    // --------------------------------------------------------
    // BLACK LINE / WHITE BACKGROUND
    // --------------------------------------------------------

    if (
        INVERT_LINE_FOLLOWING
    )
    {
        value =
            1000 -
            value;
    }


    return
        (int)value;
}


// ============================================================
// UPDATE
// ============================================================
//
// Reads all 16 sensors and calculates:
//
// - normalized strength
// - active sensor count
// - total line strength
// - weighted line position
// - line detected
// ============================================================

void Sensors::update()
{
    if (!calibrated)
    {
        return;
    }


    long weightedSum = 0;


    totalLineStrengthValue =
        0;


    activeSensorCount =
        0;


    // --------------------------------------------------------
    // READ C0 → C15
    // --------------------------------------------------------

    for (
        int sensor = 0;
        sensor < SENSOR_COUNT;
        sensor++
    )
    {
        rawValue[sensor] =
            readChannel(
                sensor
            );


        lineStrengthValue[sensor] =
            normalize(
                sensor
            );


        // ----------------------------------------------------
        // Active sensor
        // ----------------------------------------------------

        if (
            lineStrengthValue[sensor]
            >=
            LINE_STRENGTH_THRESHOLD
        )
        {
            activeSensorCount++;
        }


        // ----------------------------------------------------
        // Ignore very weak readings
        // ----------------------------------------------------

        int strength =
            lineStrengthValue[sensor];


        if (
            strength < 80
        )
        {
            strength = 0;
        }


        totalLineStrengthValue +=
            strength;


        weightedSum +=
            (
                (long)
                sensorWeight[sensor]
            )
            *
            strength;
    }


    // --------------------------------------------------------
    // LINE DETECTION
    // --------------------------------------------------------

    lineDetectedValue =
        (
            totalLineStrengthValue
            >=
            MIN_LINE_STRENGTH_SUM
        );


    // --------------------------------------------------------
    // LINE POSITION
    // --------------------------------------------------------

    if (
        lineDetectedValue
        &&
        totalLineStrengthValue > 0
    )
    {
        linePositionValue =
            weightedSum /
            totalLineStrengthValue;
    }
}


// ============================================================
// RAW VALUE
// ============================================================

int Sensors::raw(
    uint8_t sensor
)
{
    if (
        sensor >=
        SENSOR_COUNT
    )
    {
        return 0;
    }


    return
        rawValue[sensor];
}


// ============================================================
// LINE STRENGTH
// ============================================================

int Sensors::strength(
    uint8_t sensor
)
{
    if (
        sensor >=
        SENSOR_COUNT
    )
    {
        return 0;
    }


    return
        lineStrengthValue[sensor];
}


// ============================================================
// THRESHOLD
// ============================================================

int Sensors::threshold(
    uint8_t sensor
)
{
    if (
        sensor >=
        SENSOR_COUNT
    )
    {
        return 0;
    }


    return
        thresholdValue[sensor];
}


// ============================================================
// WHITE VALUE
// ============================================================

int Sensors::white(
    uint8_t sensor
)
{
    if (
        sensor >=
        SENSOR_COUNT
    )
    {
        return 0;
    }


    return
        whiteValue[sensor];
}


// ============================================================
// BLACK VALUE
// ============================================================

int Sensors::black(
    uint8_t sensor
)
{
    if (
        sensor >=
        SENSOR_COUNT
    )
    {
        return 0;
    }


    return
        blackValue[sensor];
}


// ============================================================
// POSITION
// ============================================================

int Sensors::position()
{
    return
        linePositionValue;
}


// ============================================================
// ACTIVE SENSOR COUNT
// ============================================================

int Sensors::activeCount()
{
    return
        activeSensorCount;
}


// ============================================================
// TOTAL LINE STRENGTH
// ============================================================

long Sensors::totalStrength()
{
    return
        totalLineStrengthValue;
}


// ============================================================
// LINE DETECTED
// ============================================================

bool Sensors::lineDetected()
{
    return
        lineDetectedValue;
}


// ============================================================
// CALIBRATION STATUS
// ============================================================

bool Sensors::isCalibrated()
{
    return
        calibrated;
}


// ============================================================
// DEBUG RAW
// ============================================================

void Sensors::printRaw()
{
    Serial.print(
        "RAW: "
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
            Serial.print(
                ", "
            );
        }
    }


    Serial.println();
}


// ============================================================
// DEBUG STRENGTH
// ============================================================

void Sensors::printStrength()
{
    Serial.print(
        "STR: "
    );


    for (
        int i = 0;
        i < SENSOR_COUNT;
        i++
    )
    {
        Serial.print(
            lineStrengthValue[i]
        );


        if (
            i <
            SENSOR_COUNT - 1
        )
        {
            Serial.print(
                ", "
            );
        }
    }


    Serial.println();


    Serial.print(
        "POSITION = "
    );

    Serial.print(
        linePositionValue
    );


    Serial.print(
        " | ACTIVE = "
    );

    Serial.print(
        activeSensorCount
    );


    Serial.print(
        " | TOTAL = "
    );

    Serial.println(
        totalLineStrengthValue
    );
}


// ============================================================
// DEBUG CALIBRATION
// ============================================================

void Sensors::printCalibration()
{
    Serial.println();

    Serial.println(
        "========================================"
    );

    Serial.println(
        "CALIBRATION TABLE"
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

        Serial.println(
            whiteValue[i]
            -
            blackValue[i]
        );
    }
}