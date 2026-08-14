#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#include "Config.h"

class ToFSensors
{
public:
    bool begin();
    bool update();

    bool isReady() const;

    int leftMM() const;
    int frontMM() const;
    int rightMM() const;

    bool leftValid() const;
    bool frontValid() const;
    bool rightValid() const;

private:
    bool selectChannel(uint8_t channel);
    int readSensor(
        Adafruit_VL53L0X &sensor,
        uint8_t channel,
        int offsetMM
    );

    int filterValue(int rawMM, float &filteredMM, bool &hasFilteredValue);

    Adafruit_VL53L0X leftSensor;
    Adafruit_VL53L0X frontSensor;
    Adafruit_VL53L0X rightSensor;

    bool ready = false;

    int leftDistanceMM = -1;
    int frontDistanceMM = -1;
    int rightDistanceMM = -1;

    bool leftReadingValid = false;
    bool frontReadingValid = false;
    bool rightReadingValid = false;

    float leftFilteredMM = 0.0f;
    float frontFilteredMM = 0.0f;
    float rightFilteredMM = 0.0f;

    bool leftFilterStarted = false;
    bool frontFilterStarted = false;
    bool rightFilterStarted = false;

    unsigned long lastUpdateTime = 0;
};

extern ToFSensors tofSensors;
