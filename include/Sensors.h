#pragma once

#include <Arduino.h>
#include "Config.h"

// ============================================================
// SENSOR MODULE
// ============================================================
//
// Handles:
// - 16 IR sensors
// - MUX channel selection
// - Raw ADC readings
// - Individual white calibration
// - Individual black calibration
// - Individual thresholds
// - Sensor normalization
// - Line strength
// - Line position
//
// C0 → C15 = LEFT → RIGHT
// C7 + C8 = CENTER
// ============================================================

class Sensors
{
public:

    // --------------------------------------------------------
    // Start sensor system
    // --------------------------------------------------------

    void begin();


    // --------------------------------------------------------
    // Calibrate all 16 sensors
    //
    // Calibration happens only once.
    //
    // Sequence:
    // WHITE → BLACK → LOCK
    // --------------------------------------------------------

    bool calibrate();


    // --------------------------------------------------------
    // Read all sensors
    // --------------------------------------------------------

    void update();


    // --------------------------------------------------------
    // Raw ADC value
    // --------------------------------------------------------

    int raw(
        uint8_t sensor
    );


    // --------------------------------------------------------
    // Normalized line strength
    //
    // Current development mode:
    //
    // BLACK LINE  = HIGH strength
    // WHITE       = LOW strength
    // --------------------------------------------------------

    int strength(
        uint8_t sensor
    );


    // --------------------------------------------------------
    // Individual threshold
    // --------------------------------------------------------

    int threshold(
        uint8_t sensor
    );


    // --------------------------------------------------------
    // Calibration values
    // --------------------------------------------------------

    int white(
        uint8_t sensor
    );

    int black(
        uint8_t sensor
    );


    // --------------------------------------------------------
    // Line information
    // --------------------------------------------------------

    int position();

    int activeCount();

    long totalStrength();

    bool lineDetected();


    // --------------------------------------------------------
    // Calibration status
    // --------------------------------------------------------

    bool isCalibrated();


    // --------------------------------------------------------
    // Debug output
    // --------------------------------------------------------

    void printRaw();

    void printStrength();

    void printCalibration();


private:

    // ========================================================
    // MUX
    // ========================================================

    void selectChannel(
        uint8_t channel
    );


    int readChannel(
        uint8_t channel
    );


    // ========================================================
    // CALIBRATION
    // ========================================================

    void calibrateWhite();

    void calibrateBlack();

    bool createCalibration();


    // ========================================================
    // NORMALIZATION
    // ========================================================

    int normalize(
        uint8_t sensor
    );


    // ========================================================
    // SENSOR DATA
    // ========================================================

    int rawValue[SENSOR_COUNT];

    int whiteValue[SENSOR_COUNT];

    int blackValue[SENSOR_COUNT];

    int thresholdValue[SENSOR_COUNT];

    int lineStrengthValue[SENSOR_COUNT];

    bool sensorValid[SENSOR_COUNT];


    // ========================================================
    // LINE DATA
    // ========================================================

    int linePositionValue;

    int activeSensorCount;

    long totalLineStrengthValue;

    bool lineDetectedValue;


    // ========================================================
    // CALIBRATION STATUS
    // ========================================================

    bool calibrated;
};


// ============================================================
// GLOBAL SENSOR OBJECT
// ============================================================

extern Sensors sensors;