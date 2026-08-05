#include <Arduino.h>
#include "Robot.h"

Robot robot;

void setup()
{
    Serial.begin(SERIAL_BAUD);

    robot.begin();

    delay(1000);

    Serial.println("===== ROSCO Line Follower =====");
    Serial.println("Starting Individual Sensor Calibration...");
    Serial.println("Follow the prompts in Serial Monitor.");

    robot.calibrate();

    Serial.println("All sensors calibrated!");
    delay(1000);
}

void loop()
{
    robot.followLine();
}