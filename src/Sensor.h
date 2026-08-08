// ============================================================
// Sensor.h  —  16-channel analog MUX sensor driver
// ============================================================
// Hardware:
//   16-channel IR sensor array via analog multiplexer.
//   MUX select lines S0-S3 -> GPIO 0-3
//   MUX SIG output         -> GPIO 6  (ADC)
//   MUX E (enable)         -> GND (hardwired, no GPIO used)
//
// Channel addressing:
//   S3 S2 S1 S0
//    0  0  0  0 = C0  (far LEFT,  default)
//    0  0  0  1 = C1
//    ...
//    1  1  1  1 = C15 (far RIGHT, default)
//
// Sensor reversal:
//   When SENSOR_REVERSED == true, C0 maps to far RIGHT.
//
// Individual calibration:
//   Every sensor C0-C15 has a different raw ADC response.
//   This is expected hardware behaviour, NOT a fault.
//   Each sensor is calibrated independently and stored in:
//     calibration[i].whiteValue  — average ADC on WHITE surface
//     calibration[i].blackValue  — average ADC on BLACK surface
//     calibration[i].threshold   — midpoint threshold (raw ADC)
//     calibration[i].valid       — passed validation flag
//
// All consumer code works on lineStrength[i] (0-1000):
//   1000 = line
//      0 = background
// LINE_IS_WHITE controls the polarity conversion ONLY here.
// PID, position and motor code never see raw ADC values.
// ============================================================

#pragma once
#include <Arduino.h>
#include "Config.h"

// ============================================================
// SensorCalibration — per-sensor calibration data
// ============================================================
// One instance per physical sensor channel (C0-C15).
// DO NOT use a single shared blackValue or whiteValue for all
// sensors.  Each sensor has a different electrical response.
// ============================================================
struct SensorCalibration
{
    int  blackValue;   // average raw ADC over BLACK surface
    int  whiteValue;   // average raw ADC over WHITE surface
    int  threshold;    // midpoint in raw ADC: (white + black) / 2
    bool valid;        // true if abs(white - black) >= MIN_CALIBRATION_RANGE
};

// ============================================================
// Sensor class
// ============================================================

class Sensor
{
public:
    // ---- Per-sensor calibration data (one entry per channel) -
    // Access as calibration[i].whiteValue etc.
    SensorCalibration calibration[SENSOR_COUNT];

    // ---- Runtime data ----------------------------------------

    // Raw ADC values from the MUX, averaged over SENSOR_AVERAGE_SAMPLES
    int rawValues[SENSOR_COUNT];

    // Normalised line-strength after calibration + polarity mapping.
    // 0 = background,  1000 = line.
    // Meaning is the same regardless of LINE_IS_WHITE or ADC polarity.
    int lineStrength[SENSOR_COUNT];

    // Exponential-moving-average state (per sensor, for run-time filtering)
    float emaState[SENSOR_COUNT];

    // 16-bit sensor pattern  (bit i = 1 if sensor i is on line)
    uint16_t pattern;

    // Last computed weighted position (-7500 to +7500)
    int lastPosition;

    // Number of sensors currently above their individual threshold
    int activeSensorCount;

    // ---- Construction ----------------------------------------
    Sensor();

    // ---- Lifecycle -------------------------------------------

    // Initialise GPIO, ADC, MUX.  Call once from setup().
    void begin();

    // ---- Calibration -----------------------------------------

    // Full automatic white+black individual sensor calibration.
    // Call ONLY from setup().  NEVER call from loop().
    // Returns true only if ALL sensors pass validation.
    // On failure: prints failed sensor numbers; caller must keep
    // STBY LOW and halt autonomous operation.
    bool calibrate();

    // Save calibration array to ESP32 NVS flash (competition mode).
    bool saveCalibration();

    // Load calibration array from ESP32 NVS flash (competition mode).
    bool loadCalibration();

    // ---- Sensor update (call every control loop) -------------

    // Read all 16 channels through the MUX, apply EMA,
    // compute lineStrength[], pattern, activeSensorCount.
    void update();

    // ---- Derived values --------------------------------------

    // True if at least MIN_ACTIVE_SENSORS are above their threshold
    // and at least one exceeds NOISE_THRESHOLD.
    bool lineDetected() const;

    // Weighted centre-of-mass position.
    // Returns lastPosition if line is lost.
    int getPosition();

    // Return the 16-bit pattern
    uint16_t getPattern() const { return pattern; }

    // Count active sensors in a range [start, end] (inclusive)
    int countActive(int start, int end) const;

    // ---- Debug printing --------------------------------------
    void printRaw()         const;
    void printLineStrength()const;
    void printPattern()     const;
    void printCalibration() const;   // full per-sensor calibration table
    void printSensorTest()  const;   // complete sensor-test report

    // Check calibration status
    bool isCalibrated() const { return _calibrationCompleted; }

    // Reset EMA states to uninitialized
    void resetEma();

private:
    bool _emaInitialized;
    bool _calibrationCompleted;

    // MUX channel select: sets S3-S0 bits, waits MUX_SETTLE_US
    void selectChannel(uint8_t ch);

    // Read SIG for a given channel with ADC averaging; returns raw ADC
    int readChannel(uint8_t ch);

    // Read all 16 channels sequentially; fills rawValues[]
    void readAllRaw();

    // Convert rawValues[] -> lineStrength[] using per-sensor calibration.
    // Handles both ADC polarities automatically.
    // Applies LINE_IS_WHITE polarity inversion.
    // Applies EMA smoothing.
    void normalise();

    // Compute pattern and activeSensorCount from lineStrength[]
    void updatePattern();

    // Physical-to-logical channel mapping (handles SENSOR_REVERSED)
    int physicalIndex(int logicalIndex) const;
};
