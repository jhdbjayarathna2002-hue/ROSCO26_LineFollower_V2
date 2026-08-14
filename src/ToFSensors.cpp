#include "ToFSensors.h"

ToFSensors tofSensors;

bool ToFSensors::selectChannel(uint8_t channel)
{
    if (channel > 7)
    {
        return false;
    }

    Wire.beginTransmission(TCA9548A_ADDRESS);
    Wire.write(1U << channel);

    return Wire.endTransmission() == 0;
}

bool ToFSensors::begin()
{
    ready = false;

    Wire.begin(TOF_SDA_PIN, TOF_SCL_PIN);
    Wire.setClock(400000);

    delay(20);

    Wire.beginTransmission(TCA9548A_ADDRESS);
    if (Wire.endTransmission() != 0)
    {
        Serial.println("[TOF] TCA9548A not found at 0x70");
        return false;
    }

    if (!selectChannel(TOF_LEFT_CHANNEL) ||
        !leftSensor.begin(TOF_DEFAULT_ADDRESS, false, &Wire))
    {
        Serial.println("[TOF] Left VL53L0X failed on TCA channel 0");
        return false;
    }

    if (!selectChannel(TOF_FRONT_CHANNEL) ||
        !frontSensor.begin(TOF_DEFAULT_ADDRESS, false, &Wire))
    {
        Serial.println("[TOF] Front VL53L0X failed on TCA channel 1");
        return false;
    }

    if (!selectChannel(TOF_RIGHT_CHANNEL) ||
        !rightSensor.begin(TOF_DEFAULT_ADDRESS, false, &Wire))
    {
        Serial.println("[TOF] Right VL53L0X failed on TCA channel 2");
        return false;
    }

    ready = true;
    lastUpdateTime = 0;

    Serial.println("[TOF] TCA9548A and all three VL53L0X sensors ready");
    return true;
}

int ToFSensors::readSensor(
    Adafruit_VL53L0X &sensor,
    uint8_t channel,
    int offsetMM
)
{
    if (!selectChannel(channel))
    {
        return -1;
    }

    VL53L0X_RangingMeasurementData_t measurement;
    sensor.rangingTest(&measurement, false);

    if (measurement.RangeStatus == 4)
    {
        return -1;
    }

    int distance = (int)measurement.RangeMilliMeter + offsetMM;

    if (distance < 20 || distance > 2000)
    {
        return -1;
    }

    return distance;
}

int ToFSensors::filterValue(
    int rawMM,
    float &filteredMM,
    bool &hasFilteredValue
)
{
    if (rawMM < 0)
    {
        return -1;
    }

    if (!hasFilteredValue)
    {
        filteredMM = (float)rawMM;
        hasFilteredValue = true;
    }
    else
    {
        filteredMM =
            TOF_FILTER_ALPHA * (float)rawMM +
            (1.0f - TOF_FILTER_ALPHA) * filteredMM;
    }

    return (int)(filteredMM + 0.5f);
}

bool ToFSensors::update()
{
    if (!ready)
    {
        return false;
    }

    unsigned long now = millis();

    if (lastUpdateTime != 0 &&
        now - lastUpdateTime < (unsigned long)TASK2_TOF_UPDATE_MS)
    {
        return false;
    }

    lastUpdateTime = now;

    int rawLeft = readSensor(
        leftSensor,
        TOF_LEFT_CHANNEL,
        TOF_LEFT_OFFSET_MM
    );

    int rawFront = readSensor(
        frontSensor,
        TOF_FRONT_CHANNEL,
        TOF_FRONT_OFFSET_MM
    );

    int rawRight = readSensor(
        rightSensor,
        TOF_RIGHT_CHANNEL,
        TOF_RIGHT_OFFSET_MM
    );

    leftReadingValid = rawLeft >= 0;
    frontReadingValid = rawFront >= 0;
    rightReadingValid = rawRight >= 0;

    leftDistanceMM = filterValue(
        rawLeft,
        leftFilteredMM,
        leftFilterStarted
    );

    frontDistanceMM = filterValue(
        rawFront,
        frontFilteredMM,
        frontFilterStarted
    );

    rightDistanceMM = filterValue(
        rawRight,
        rightFilteredMM,
        rightFilterStarted
    );

    return true;
}

bool ToFSensors::isReady() const
{
    return ready;
}

int ToFSensors::leftMM() const
{
    return leftDistanceMM;
}

int ToFSensors::frontMM() const
{
    return frontDistanceMM;
}

int ToFSensors::rightMM() const
{
    return rightDistanceMM;
}

bool ToFSensors::leftValid() const
{
    return leftReadingValid;
}

bool ToFSensors::frontValid() const
{
    return frontReadingValid;
}

bool ToFSensors::rightValid() const
{
    return rightReadingValid;
}
