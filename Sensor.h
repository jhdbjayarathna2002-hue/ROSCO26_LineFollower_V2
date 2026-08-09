#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>
#include "Config.h"

class Sensor
{
public:

    void begin();
    void calibrate();

    void readRaw();
    void readNormalized();

    bool lineDetected();

    int getPosition();

    // -------- NEW --------
    uint8_t getPattern();

    void printRaw();
    void printNormalized();

private:

    int raw[SENSOR_COUNT];
    int value[SENSOR_COUNT];

    int sensorMin[SENSOR_COUNT];
    int sensorMax[SENSOR_COUNT];
    int sensorThreshold[SENSOR_COUNT];

    int lastPosition = 0;
};

#endif