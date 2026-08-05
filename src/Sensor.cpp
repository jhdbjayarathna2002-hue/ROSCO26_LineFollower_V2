#include "Sensor.h"

//--------------------------------------------------
// Initialize Sensor Pins
//--------------------------------------------------

void Sensor::begin()
{
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        pinMode(SENSOR_PINS[i], INPUT);

        sensorMin[i] = 4095;
        sensorMax[i] = 0;
        sensorThreshold[i] = LINE_THRESHOLD;  // default until calibrated
    }
}

//--------------------------------------------------
// Read Raw Sensor Values
//--------------------------------------------------

void Sensor::readRaw()
{
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        raw[i] = analogRead(SENSOR_PINS[i]);
    }
}

//--------------------------------------------------
// Calibrate Sensors (Individually)
//--------------------------------------------------

// NeoPixel helpers
static void ledOff()  { rgbLedWrite(LED_PIN, 0, 0, 0); }
static void ledOn()   { rgbLedWrite(LED_PIN, 0, LED_BRIGHTNESS, 0); }  // green = sampling
static void ledBlink(){ rgbLedWrite(LED_PIN, 0, 0, LED_BRIGHTNESS); }  // blue  = get ready

// Blink countdown: starts slow, speeds up, then LED stays ON (green)
static void blinkCountdown(unsigned long durationMs)
{
    unsigned long start = millis();
    bool ledState = false;

    while (millis() - start < durationMs)
    {
        // Progress 0.0 → 1.0
        float progress = (float)(millis() - start) / durationMs;

        // Blink interval: 400ms → 50ms (speeds up)
        int interval = 400 - (int)(350 * progress);
        if (interval < 50) interval = 50;

        ledState = !ledState;
        if (ledState) ledBlink(); else ledOff();
        delay(interval);
    }

    // Solid green = now sampling
    ledOn();
}

void Sensor::calibrate()
{
    // Turn off NeoPixel
    ledOff();

    Serial.println("=================================");
    Serial.println("Sensor Calibration");
    Serial.println("LED blinks = get ready");
    Serial.println("LED solid  = sampling now");
    Serial.println("=================================");

    // Temp arrays for averages
    long whiteSum[SENSOR_COUNT] = {0};
    int  whiteMin[SENSOR_COUNT];
    int  whiteMax[SENSOR_COUNT];

    long lineSum[SENSOR_COUNT]  = {0};
    int  lineMin[SENSOR_COUNT];
    int  lineMax[SENSOR_COUNT];

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        whiteMin[i] = 4095;  whiteMax[i] = 0;
        lineMin[i]  = 4095;  lineMax[i]  = 0;
    }

    // ======== STEP 1: WHITE surface ========

    Serial.println("---------------------------------");
    Serial.println(">> Place ALL sensors on WHITE surface...");

    blinkCountdown(CALIBRATION_COUNTDOWN);

    // Sample all sensors
    for (int s = 0; s < CALIBRATION_SAMPLES; s++)
    {
        for (int i = 0; i < SENSOR_COUNT; i++)
        {
            int reading = analogRead(SENSOR_PINS[i]);

            whiteSum[i] += reading;
            if (reading < whiteMin[i]) whiteMin[i] = reading;
            if (reading > whiteMax[i]) whiteMax[i] = reading;
        }
        delay(5);
    }

    // Done sampling — LED off
    ledOff();

    Serial.println("White surface sampled:");
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        Serial.print("  S");
        Serial.print(i + 1);
        Serial.print("  Avg: ");
        Serial.print((int)(whiteSum[i] / CALIBRATION_SAMPLES));
        Serial.print("  Min: ");
        Serial.print(whiteMin[i]);
        Serial.print("  Max: ");
        Serial.println(whiteMax[i]);
    }

    // ======== STEP 2: LINE (dark) surface ========

    Serial.println("---------------------------------");
    Serial.println(">> Place ALL sensors on LINE (dark) surface...");

    blinkCountdown(CALIBRATION_COUNTDOWN);

    // Sample all sensors
    for (int s = 0; s < CALIBRATION_SAMPLES; s++)
    {
        for (int i = 0; i < SENSOR_COUNT; i++)
        {
            int reading = analogRead(SENSOR_PINS[i]);

            lineSum[i] += reading;
            if (reading < lineMin[i]) lineMin[i] = reading;
            if (reading > lineMax[i]) lineMax[i] = reading;
        }
        delay(5);
    }

    // Done sampling — LED off
    ledOff();

    Serial.println("Line surface sampled:");
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        Serial.print("  S");
        Serial.print(i + 1);
        Serial.print("  Avg: ");
        Serial.print((int)(lineSum[i] / CALIBRATION_SAMPLES));
        Serial.print("  Min: ");
        Serial.print(lineMin[i]);
        Serial.print("  Max: ");
        Serial.println(lineMax[i]);
    }

    // ======== STEP 3: Compute per-sensor min, max, threshold ========

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        sensorMin[i] = min(whiteMin[i], lineMin[i]);
        sensorMax[i] = max(whiteMax[i], lineMax[i]);

        // Threshold = midpoint of white avg and line avg
        // mapped into normalized 0-1000 range
        int whiteAvg = (int)(whiteSum[i] / CALIBRATION_SAMPLES);
        int lineAvg  = (int)(lineSum[i]  / CALIBRATION_SAMPLES);
        int rawMid   = (whiteAvg + lineAvg) / 2;

        if (sensorMax[i] != sensorMin[i])
        {
            sensorThreshold[i] = map(rawMid,
                                     sensorMin[i],
                                     sensorMax[i],
                                     0, 1000);
            sensorThreshold[i] = constrain(sensorThreshold[i], 100, 900);
        }
        else
        {
            sensorThreshold[i] = LINE_THRESHOLD;  // fallback
        }
    }

    // ---- Summary ----

    Serial.println("=================================");
    Serial.println("Calibration Complete");
    Serial.println("---------------------------------");

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        Serial.print("S");
        Serial.print(i + 1);

        Serial.print("  Min: ");
        Serial.print(sensorMin[i]);

        Serial.print("  Max: ");
        Serial.print(sensorMax[i]);

        Serial.print("  Thresh: ");
        Serial.println(sensorThreshold[i]);
    }

    Serial.println("=================================");

    // Make sure LED is off after calibration
    ledOff();
}

//--------------------------------------------------
// Read Normalized Sensor Values
//--------------------------------------------------

void Sensor::readNormalized()
{
    readRaw();

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        if (sensorMax[i] == sensorMin[i])
        {
            value[i] = 0;
            continue;
        }

        value[i] = map(raw[i],
                       sensorMin[i],
                       sensorMax[i],
                       0,
                       1000);

        value[i] = constrain(value[i], 0, 1000);
    }
}

//--------------------------------------------------
// Detect Line
//--------------------------------------------------

bool Sensor::lineDetected()
{
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        if (value[i] > sensorThreshold[i])
            return true;
    }

    return false;
}

//--------------------------------------------------
// Calculate Line Position
//--------------------------------------------------

int Sensor::getPosition()
{
    const int weight[SENSOR_COUNT] =
    {
        -3500,
        -2500,
        -1500,
         -500,
          500,
         1500,
         2500,
         3500
    };

    long numerator = 0;
    long denominator = 0;

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        if (value[i] > sensorThreshold[i])
        {
            numerator += (long)value[i] * weight[i];
            denominator += value[i];
        }
    }

    if (denominator == 0)
    {
        return lastPosition;
    }

    lastPosition = numerator / denominator;

    return lastPosition;
}

//--------------------------------------------------
// Print Raw Sensor Values
//--------------------------------------------------

void Sensor::printRaw()
{
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        Serial.print(raw[i]);
        Serial.print('\t');
    }

    Serial.println();
}

//--------------------------------------------------
// Print Normalized Sensor Values
//--------------------------------------------------

void Sensor::printNormalized()
{
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        Serial.print(value[i]);
        Serial.print('\t');
    }

    Serial.println();
}
//--------------------------------------------------
// Return 8-bit Sensor Pattern
//--------------------------------------------------

uint8_t Sensor::getPattern()
{
    uint8_t pattern = 0;

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        if (value[i] > sensorThreshold[i])
        {
            pattern |= (1 << i);
        }
    }

    return pattern;
}