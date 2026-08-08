// ============================================================
// Sensor.cpp  —  16-channel analog MUX sensor driver
// ============================================================
// Individual per-sensor calibration.
//
// Every sensor C0-C15 has a different raw ADC response.
// This is expected hardware behaviour and is handled by
// calibrating each sensor independently.
//
// Example measured values that demonstrate why individual
// calibration is required (NOT hard-coded constants):
//
//   C0:  BLACK~60    WHITE~684     range~624
//   C1:  BLACK~390   WHITE~3021    range~2631
//   C9:  BLACK~1680  WHITE~3067    range~1387
//   C15: BLACK~91    WHITE~3055    range~2964
//
// The firmware measures these values at startup automatically.
//
// Normalization formula (per sensor, per spec section 13):
//   If whiteValue > blackValue:
//     whiteStrength = ((raw - blackValue) * 1000) / (whiteValue - blackValue)
//   If blackValue > whiteValue:
//     whiteStrength = ((raw - whiteValue) * 1000) / (blackValue - whiteValue)
//   constrain whiteStrength to 0-1000
//   if LINE_IS_WHITE: lineStrength = whiteStrength
//   if not:           lineStrength = 1000 - whiteStrength
// ============================================================

#include "Sensor.h"
#include <Preferences.h>    // ESP32 NVS (non-volatile storage)

// ============================================================
// RGB LED helpers  (built-in WS2812 on GPIO RGB_LED_PIN)
// ============================================================
// BLUE  (blinking) = waiting — place sensors on surface
// GREEN (solid)    = sampling in progress
// GREEN (solid 3s) = calibration PASSED — motors will start
// RED   (blinking) = calibration FAILED — check Serial Monitor
// OFF              = idle / done
// ============================================================

static void ledOff()
{
    rgbLedWrite(RGB_LED_PIN, 0, 0, 0);
}

static void ledGreen()
{
    rgbLedWrite(RGB_LED_PIN, 0, RGB_LED_BRIGHTNESS, 0);
}

static void ledBlue()
{
    rgbLedWrite(RGB_LED_PIN, 0, 0, RGB_LED_BRIGHTNESS);
}

static void ledRed()
{
    rgbLedWrite(RGB_LED_PIN, RGB_LED_BRIGHTNESS, 0, 0);
}

// blinkBlue()  — blink the LED blue for durationMs ms.
// Blink interval starts at 400 ms and speeds up to 50 ms
// as the countdown approaches zero, giving a visual urgency cue.
static void blinkBlue(unsigned long durationMs)
{
    unsigned long t0    = millis();
    bool          state = false;

    while (millis() - t0 < durationMs)
    {
        float progress = (float)(millis() - t0) / (float)durationMs;
        int   interval = 400 - (int)(350.0f * progress);
        if (interval < 50) interval = 50;

        state = !state;
        if (state) ledBlue(); else ledOff();
        delay(interval);
    }

    // End the countdown with LED off (green comes on right after)
    ledOff();
}

// blinkRed()  — blink the LED red rapidly for durationMs ms.
// Used to signal calibration failure.
static void blinkRed(unsigned long durationMs)
{
    unsigned long t0    = millis();
    bool          state = false;

    while (millis() - t0 < durationMs)
    {
        state = !state;
        if (state) ledRed(); else ledOff();
        delay(150);
    }
    ledOff();
}

// ============================================================
// Static weight table for 16 channels
// C0 = -7500 (far left), C15 = +7500 (far right)
// ============================================================
static const int WEIGHTS[SENSOR_COUNT] =
{
    -7500, -6500, -5500, -4500,
    -3500, -2500, -1500,  -500,
     +500, +1500, +2500, +3500,
    +4500, +5500, +6500, +7500
};

// ============================================================
// Construction
// ============================================================

Sensor::Sensor()
    : _emaInitialized(false),
      _calibrationCompleted(false)
{
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        calibration[i].blackValue = 0;
        calibration[i].whiteValue = 0;
        calibration[i].threshold  = 0;
        calibration[i].valid      = false;

        rawValues[i]    = 0;
        lineStrength[i] = 0;
        emaState[i]     = 0.0f;
    }

    pattern           = 0;
    lastPosition      = 0;
    activeSensorCount = 0;
}

void Sensor::resetEma()
{
    _emaInitialized = false;
    for (int i = 0; i < SENSOR_COUNT; i++)
        emaState[i] = 0.0f;
}

// ============================================================
// begin()  —  initialise GPIO / ADC / MUX
// ============================================================

void Sensor::begin()
{
    // MUX select lines
    pinMode(MUX_S0, OUTPUT);
    pinMode(MUX_S1, OUTPUT);
    pinMode(MUX_S2, OUTPUT);
    pinMode(MUX_S3, OUTPUT);

    digitalWrite(MUX_S0, LOW);
    digitalWrite(MUX_S1, LOW);
    digitalWrite(MUX_S2, LOW);
    digitalWrite(MUX_S3, LOW);

    // SIG line (analog input)
    pinMode(MUX_SIG, INPUT);
    analogReadResolution(ADC_RESOLUTION_BITS);

    // Built-in RGB LED — turn off at startup
    ledOff();

    // Reset EMA state
    resetEma();
}

// ============================================================
// Private: physicalIndex()
// Maps a logical sensor index to the physical MUX channel.
// When SENSOR_REVERSED is true, the array is mirrored.
// ============================================================

int Sensor::physicalIndex(int logicalIndex) const
{
#if SENSOR_REVERSED
    return (SENSOR_COUNT - 1) - logicalIndex;
#else
    return logicalIndex;
#endif
}

// ============================================================
// Private: selectChannel()
// Set S3-S0 on the MUX and wait for settling.
// S0 = bit 0, S1 = bit 1, S2 = bit 2, S3 = bit 3
// ============================================================

void Sensor::selectChannel(uint8_t ch)
{
    digitalWrite(MUX_S0, (ch >> 0) & 0x01);
    digitalWrite(MUX_S1, (ch >> 1) & 0x01);
    digitalWrite(MUX_S2, (ch >> 2) & 0x01);
    digitalWrite(MUX_S3, (ch >> 3) & 0x01);

    delayMicroseconds(MUX_SETTLE_US);
}

// ============================================================
// Private: readChannel()
// Select a MUX channel, then average SENSOR_AVERAGE_SAMPLES reads.
// Returns the averaged raw 12-bit ADC value (0-4095).
// ============================================================

int Sensor::readChannel(uint8_t ch)
{
    selectChannel(ch);

    long sum = 0;
    for (int s = 0; s < SENSOR_AVERAGE_SAMPLES; s++)
        sum += analogRead(MUX_SIG);

    return (int)(sum / SENSOR_AVERAGE_SAMPLES);
}

// ============================================================
// Private: readAllRaw()
// Scan C0-C15 sequentially, fill rawValues[] in LOGICAL order.
// ============================================================

void Sensor::readAllRaw()
{
    for (int logical = 0; logical < SENSOR_COUNT; logical++)
    {
        int physical = physicalIndex(logical);
        rawValues[logical] = readChannel((uint8_t)physical);
    }
}

// ============================================================
// Private: normalise()
// Convert rawValues[] -> lineStrength[] using per-sensor calibration.
//
// Per spec section 13:
//
//   Case A  (white ADC > black ADC):
//     whiteStrength = ((raw - blackValue) * 1000) / (whiteValue - blackValue)
//
//   Case B  (black ADC > white ADC, inverted polarity):
//     whiteStrength = ((blackValue - raw) * 1000) / (blackValue - whiteValue)
//
//   constrain to 0-1000
//
//   if LINE_IS_WHITE: lineStrength = whiteStrength
//   else:             lineStrength = 1000 - whiteStrength
//
// Then apply EMA smoothing (initialized with first reading to prevent cold start lag).
// ============================================================

void Sensor::normalise()
{
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        // Invalid sensors output 0 (background) so they are ignored
        if (!calibration[i].valid)
        {
            lineStrength[i] = 0;
            continue;
        }

        const int white = calibration[i].whiteValue;
        const int black = calibration[i].blackValue;
        const int raw   = rawValues[i];

        int whiteStrength;

        if (white > black)
        {
            // Case A: higher ADC = more white
            int span   = white - black;
            int clamped = constrain(raw, black, white);
            whiteStrength = (int)(((long)(clamped - black) * 1000L) / span);
        }
        else
        {
            // Case B: lower ADC = more white (inverted polarity)
            // When raw == white, (black - white) / span * 1000 = 1000
            // When raw == black, (black - black) / span * 1000 = 0
            int span    = black - white;
            int clamped = constrain(raw, white, black);
            whiteStrength = (int)(((long)(black - clamped) * 1000L) / span);
        }

        whiteStrength = constrain(whiteStrength, 0, 1000);

        // Apply track-colour polarity (LINE_IS_WHITE is compile-time)
#if LINE_IS_WHITE
        int ls = whiteStrength;
#else
        int ls = 1000 - whiteStrength;
#endif

        // Apply EMA smoothing (seed with first reading on initial run)
        if (!_emaInitialized)
        {
            emaState[i] = (float)ls;
        }
        else
        {
            emaState[i] = SENSOR_EMA_ALPHA * (float)ls
                        + (1.0f - SENSOR_EMA_ALPHA) * emaState[i];
        }

        lineStrength[i] = (int)emaState[i];
    }
    _emaInitialized = true;
}

// ============================================================
// Private: updatePattern()
// Build the 16-bit pattern and count active sensors.
// Uses each sensor's individual threshold (in lineStrength space).
// threshold is stored as raw ADC midpoint but lineStrength
// comparison uses the normalised midpoint at 500.
// ============================================================

void Sensor::updatePattern()
{
    pattern           = 0;
    activeSensorCount = 0;

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        // lineStrength midpoint is always 500 (normalised scale)
        if (lineStrength[i] >= LINE_THRESHOLD)
        {
            pattern |= (uint16_t)(1U << i);
            activeSensorCount++;
        }
    }
}

// ============================================================
// update()  —  full sensor read cycle (call every loop)
// ============================================================

void Sensor::update()
{
    readAllRaw();
    normalise();
    updatePattern();
}

// ============================================================
// lineDetected()
// ============================================================

bool Sensor::lineDetected() const
{
    if (activeSensorCount < MIN_ACTIVE_SENSORS)
        return false;

    // At least one sensor must exceed NOISE_THRESHOLD
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        if (lineStrength[i] > NOISE_THRESHOLD)
            return true;
    }

    return false;
}

// ============================================================
// getPosition()
// Weighted centre-of-mass calculation.
// Sensors below NOISE_THRESHOLD are excluded from the sum.
// Returns lastPosition if line is lost (denominator == 0).
// ============================================================

int Sensor::getPosition()
{
    long numerator   = 0;
    long denominator = 0;

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        if (lineStrength[i] <= NOISE_THRESHOLD)
            continue;

        numerator   += (long)lineStrength[i] * WEIGHTS[i];
        denominator += lineStrength[i];
    }

    if (denominator == 0)
        return lastPosition;   // line lost: hold last known position

    lastPosition = (int)(numerator / denominator);
    return lastPosition;
}

// ============================================================
// countActive()  —  count active sensors in range [start, end]
// ============================================================

int Sensor::countActive(int start, int end) const
{
    int count = 0;

    for (int i = start; i <= end && i < SENSOR_COUNT; i++)
    {
        if (lineStrength[i] >= LINE_THRESHOLD)
            count++;
    }

    return count;
}

// ============================================================
// calibrate()
// ============================================================
// Individual per-sensor calibration.
// Called ONCE from setup().  NEVER called from loop().
//
// Startup sequence (per spec section 10):
//   STBY LOW (motor driver disabled, caller's responsibility)
//   -> STEP 1: sample WHITE surface for all C0-C15
//   -> STEP 2: sample BLACK surface for all C0-C15
//   -> STEP 3: compute whiteAverage[], blackAverage[]
//   -> STEP 4: determine polarity and compute range
//   -> STEP 5: compute individual threshold per sensor
//   -> STEP 6: validate each sensor
//   -> print calibration report
//   -> return true only if all sensors valid
//
// Returns false if any sensor fails validation.
// On false: caller must keep STBY LOW and halt autonomy.
// ============================================================

bool Sensor::calibrate()
{
    if (_calibrationCompleted)
    {
        Serial.println(F("[SENSOR] Calibration already completed for this power cycle. Skipping."));
        return true;
    }

    Serial.println(F("================================================="));
    Serial.println(F(" ROSCO'26  INDIVIDUAL SENSOR CALIBRATION"));
    Serial.println(F("================================================="));
#if LINE_IS_WHITE
    Serial.println(F(" LINE COLOUR: WHITE"));
    Serial.println(F(" (WHITE line on BLACK background — ROSCO'26 official)"));
#else
    Serial.println(F(" LINE COLOUR: BLACK"));
    Serial.println(F(" (BLACK line on WHITE background — lab testing)"));
#endif
    Serial.println(F(" Each sensor C0-C15 is calibrated independently."));
    Serial.println();

    // ----------------------------------------------------------
    // Accumulation arrays (local — not stored permanently)
    // ----------------------------------------------------------
    long whiteSum[SENSOR_COUNT];
    long blackSum[SENSOR_COUNT];

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        whiteSum[i] = 0;
        blackSum[i] = 0;
    }

    // ----------------------------------------------------------
    // STEP 1:  WHITE surface sampling
    // ----------------------------------------------------------
    Serial.println(F("-------------------------------------------------"));
    Serial.println(F(" STEP 1:  Place ALL sensors on WHITE surface."));
    Serial.println(F("          Sampling begins in 3 seconds..."));
    Serial.println(F("-------------------------------------------------"));

    // Countdown: LED blinks BLUE  = "place sensors on WHITE surface"
    Serial.println(F(" LED: BLUE blinking = place sensors on WHITE surface."));
    blinkBlue((unsigned long)CALIBRATION_COUNTDOWN_MS);

    // Sampling: LED solid GREEN  = "sampling in progress"
    ledGreen();
    Serial.println();
    Serial.println(F(" LED: GREEN solid = sampling WHITE surface..."));
    Serial.println(F(" >> Sampling WHITE surface..."));

    for (int s = 0; s < CALIBRATION_SAMPLES; s++)
    {
        for (int i = 0; i < SENSOR_COUNT; i++)
        {
            int physical = physicalIndex(i);
            whiteSum[i] += readChannel((uint8_t)physical);
        }
        delay(CALIBRATION_SAMPLE_DELAY_MS);
    }

    ledOff();   // sampling done — LED off
    Serial.println(F(" >> WHITE surface sampling complete."));
    Serial.println();

    // ----------------------------------------------------------
    // STEP 2:  BLACK surface sampling
    // ----------------------------------------------------------
    Serial.println(F("-------------------------------------------------"));
    Serial.println(F(" STEP 2:  Place ALL sensors on BLACK surface."));
    Serial.println(F("          Sampling begins in 3 seconds..."));
    Serial.println(F("-------------------------------------------------"));

    // Countdown: LED blinks BLUE  = "place sensors on BLACK surface"
    Serial.println(F(" LED: BLUE blinking = place sensors on BLACK surface."));
    blinkBlue((unsigned long)CALIBRATION_COUNTDOWN_MS);

    // Sampling: LED solid GREEN  = "sampling in progress"
    ledGreen();
    Serial.println();
    Serial.println(F(" LED: GREEN solid = sampling BLACK surface..."));
    Serial.println(F(" >> Sampling BLACK surface..."));

    for (int s = 0; s < CALIBRATION_SAMPLES; s++)
    {
        for (int i = 0; i < SENSOR_COUNT; i++)
        {
            int physical = physicalIndex(i);
            blackSum[i] += readChannel((uint8_t)physical);
        }
        delay(CALIBRATION_SAMPLE_DELAY_MS);
    }

    ledOff();   // sampling done — LED off
    Serial.println(F(" >> BLACK surface sampling complete."));
    Serial.println();

    // ----------------------------------------------------------
    // STEP 3-6: Compute averages, ranges, thresholds, validate
    // ----------------------------------------------------------
    bool allValid = true;

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        // STEP 3: averages
        int whiteAvg = (int)(whiteSum[i] / (long)CALIBRATION_SAMPLES);
        int blackAvg = (int)(blackSum[i] / (long)CALIBRATION_SAMPLES);

        calibration[i].whiteValue = whiteAvg;
        calibration[i].blackValue = blackAvg;

        // STEP 4: range (polarity-agnostic absolute difference)
        int range = abs(whiteAvg - blackAvg);

        // STEP 5: individual threshold = midpoint of white and black
        calibration[i].threshold = (whiteAvg + blackAvg) / 2;

        // STEP 6: validate — sensor is valid if range >= MIN_CALIBRATION_RANGE
        if (range < MIN_CALIBRATION_RANGE)
        {
            calibration[i].valid = false;
            allValid             = false;
        }
        else
        {
            calibration[i].valid = true;
        }
    }

    // ----------------------------------------------------------
    // Print calibration report (per spec section 14)
    // ----------------------------------------------------------
    Serial.println(F("================================================="));
    Serial.println(F(" CALIBRATION REPORT"));
    Serial.println(F("================================================="));

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        int range = abs(calibration[i].whiteValue - calibration[i].blackValue);

        Serial.printf("C%d:\n", i);
        Serial.printf("  WHITE     = %d\n",  calibration[i].whiteValue);
        Serial.printf("  BLACK     = %d\n",  calibration[i].blackValue);
        Serial.printf("  RANGE     = %d\n",  range);
        Serial.printf("  THRESHOLD = %d\n",  calibration[i].threshold);
        Serial.printf("  VALID     = %s\n\n",calibration[i].valid ? "YES" : "NO");
    }

    // Print line colour
    Serial.println(F("-------------------------------------------------"));
    Serial.print(F(" LINE COLOUR: "));
#if LINE_IS_WHITE
    Serial.println(F("WHITE"));
#else
    Serial.println(F("BLACK"));
#endif
    Serial.println(F("-------------------------------------------------"));

    // Report any failures (per spec section 15)
    if (!allValid)
    {
        Serial.println(F(" [ERROR] The following sensors FAILED calibration:"));
        for (int i = 0; i < SENSOR_COUNT; i++)
        {
            if (!calibration[i].valid)
            {
                int range = abs(calibration[i].whiteValue
                              - calibration[i].blackValue);
                Serial.printf("   C%d  RANGE=%d  (minimum required: %d)\n",
                              i, range, MIN_CALIBRATION_RANGE);
            }
        }
        Serial.println(F(" Check sensor wiring and surface contrast."));
        Serial.println(F(" STBY remains LOW.  Autonomous operation NOT started."));
    }
    else
    {
        Serial.println(F(" [OK]  All 16 sensors calibrated successfully."));
    }

    Serial.println(F("================================================="));

    // Prime EMA state to zero (no cold-start transient)
    for (int i = 0; i < SENSOR_COUNT; i++)
        emaState[i] = 0.0f;

    // ----------------------------------------------------------
    // LED result indication — visible without Serial Monitor
    // ----------------------------------------------------------
    if (allValid)
    {
        // SOLID GREEN for 3 seconds = calibration passed, motors starting
        Serial.println(F(" LED: GREEN solid 3s = calibration PASSED."));
        ledGreen();
        delay(3000);
        ledOff();
    }
    else
    {
        // RAPID RED BLINK for 5 seconds = calibration failed, motors blocked
        Serial.println(F(" LED: RED blinking = calibration FAILED."));
        Serial.println(F("      Check Serial Monitor for details."));
        Serial.println(F("      Ensure sensors were on correct surface during countdown."));
        blinkRed(5000);
    }

    _calibrationCompleted = allValid;
    return allValid;
}

// ============================================================
// saveCalibration()  —  persist calibration[] to ESP32 NVS
// Architecture prepared for competition mode (spec section 17).
// ============================================================

bool Sensor::saveCalibration()
{
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, false))
    {
        Serial.println(F("[NVS] Failed to open namespace for writing."));
        return false;
    }

    // Save white and black raw ADC values; thresholds are derived
    // from them and can be recomputed, but save them for convenience.
    int  whiteArr[SENSOR_COUNT];
    int  blackArr[SENSOR_COUNT];
    int  threshArr[SENSOR_COUNT];

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        whiteArr[i]  = calibration[i].whiteValue;
        blackArr[i]  = calibration[i].blackValue;
        threshArr[i] = calibration[i].threshold;
    }

    prefs.putBytes(NVS_KEY_WHITE,  whiteArr,  sizeof(whiteArr));
    prefs.putBytes(NVS_KEY_BLACK,  blackArr,  sizeof(blackArr));
    prefs.putBytes(NVS_KEY_THRESH, threshArr, sizeof(threshArr));
    prefs.end();

    Serial.println(F("[NVS] Calibration saved to flash."));
    return true;
}

// ============================================================
// loadCalibration()  —  restore calibration[] from ESP32 NVS
// Architecture prepared for competition mode (spec section 17).
// ============================================================

bool Sensor::loadCalibration()
{
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, true))
    {
        Serial.println(F("[NVS] Failed to open namespace for reading."));
        return false;
    }

    int whiteArr[SENSOR_COUNT];
    int blackArr[SENSOR_COUNT];
    int threshArr[SENSOR_COUNT];

    size_t wLen = prefs.getBytes(NVS_KEY_WHITE,  whiteArr,  sizeof(whiteArr));
    size_t bLen = prefs.getBytes(NVS_KEY_BLACK,  blackArr,  sizeof(blackArr));
    size_t tLen = prefs.getBytes(NVS_KEY_THRESH, threshArr, sizeof(threshArr));
    prefs.end();

    if (wLen != sizeof(whiteArr)  ||
        bLen != sizeof(blackArr)  ||
        tLen != sizeof(threshArr))
    {
        Serial.println(F("[NVS] Calibration data missing or corrupt."));
        return false;
    }

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        calibration[i].whiteValue = whiteArr[i];
        calibration[i].blackValue = blackArr[i];
        calibration[i].threshold  = threshArr[i];
        calibration[i].valid      = true;  // assume stored data is good
    }

    _calibrationCompleted = true;
    Serial.println(F("[NVS] Calibration loaded from flash."));
    printCalibration();
    return true;
}

// ============================================================
// Debug printing
// ============================================================

void Sensor::printRaw() const
{
    Serial.print(F("RAW: "));
    for (int i = 0; i < SENSOR_COUNT; i++)
        Serial.printf("%4d ", rawValues[i]);
    Serial.println();
}

void Sensor::printLineStrength() const
{
    Serial.print(F("STR: "));
    for (int i = 0; i < SENSOR_COUNT; i++)
        Serial.printf("%4d ", lineStrength[i]);
    Serial.println();
}

void Sensor::printPattern() const
{
    Serial.print(F("PAT: "));
    for (int i = SENSOR_COUNT - 1; i >= 0; i--)
        Serial.print((pattern >> i) & 1);
    Serial.printf("  (0x%04X)\n", pattern);
}

void Sensor::printCalibration() const
{
    Serial.println(F(" CH   WHITE   BLACK   THRESH  VALID"));
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        int range = abs(calibration[i].whiteValue - calibration[i].blackValue);
        Serial.printf(" C%-2d  %-6d  %-6d  %-6d  %s  RANGE=%d\n",
                      i,
                      calibration[i].whiteValue,
                      calibration[i].blackValue,
                      calibration[i].threshold,
                      calibration[i].valid ? "YES" : "NO ",
                      range);
    }
}

void Sensor::printSensorTest() const
{
    Serial.println(F("========= SENSOR TEST ========="));

    // RAW ADC
    Serial.print(F("RAW:     "));
    for (int i = 0; i < SENSOR_COUNT; i++)
        Serial.printf("%4d ", rawValues[i]);
    Serial.println();

    // LINE STRENGTH (normalised 0-1000)
    Serial.print(F("STR:     "));
    for (int i = 0; i < SENSOR_COUNT; i++)
        Serial.printf("%4d ", lineStrength[i]);
    Serial.println();

    // 16-bit PATTERN
    Serial.print(F("PATTERN: "));
    for (int i = SENSOR_COUNT - 1; i >= 0; i--)
        Serial.print((pattern >> i) & 1);
    Serial.printf("  (0x%04X)\n", pattern);

    // POSITION
    Serial.printf("POSITION: %d\n", lastPosition);

    // LINE DETECTED
    Serial.printf("LINE:     %s\n", lineDetected() ? "DETECTED" : "LOST");

    // TRACK COLOUR
#if LINE_IS_WHITE
    Serial.println(F("TRACK COLOUR: WHITE LINE on BLACK"));
#else
    Serial.println(F("TRACK COLOUR: BLACK LINE on WHITE"));
#endif

    Serial.println(F("==============================="));
}