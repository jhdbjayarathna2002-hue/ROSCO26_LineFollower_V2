#include "Robot.h"

void Robot::begin()
{
    sensor.begin();
    motor.begin();
}

void Robot::calibrate()
{
    sensor.calibrate();
}

void Robot::followLine()
{
    sensor.readNormalized();

    // ---- Line lost → search by turning left ----

    if (!sensor.lineDetected())
    {
        searchLine();
        return;
    }

    // ---- Line found → PID follow ----

    int position = sensor.getPosition();

    int error = position;

    int correction = pid.compute(error);

    int leftSpeed = baseSpeed - correction;
    int rightSpeed = baseSpeed + correction;

    leftSpeed = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);

    motor.setSpeed(leftSpeed, rightSpeed);

    Serial.print("Position : ");
    Serial.print(position);

    Serial.print("\tError : ");
    Serial.print(error);

    Serial.print("\tCorrection : ");
    Serial.print(correction);

    Serial.print("\tLeft : ");
    Serial.print(leftSpeed);

    Serial.print("\tRight : ");
    Serial.println(rightSpeed);
}

//--------------------------------------------------
// Search for line: spin left until line is found
//--------------------------------------------------

void Robot::searchLine()
{
    Serial.println("LINE LOST! Searching...");

    // Use last known position to decide which way to spin.
    // getPosition() returns the cached lastPosition when no sensor is active.
    // Positive = line was to the right → turn right; negative → turn left.
    int lastPos = sensor.getPosition();

    if (lastPos > 0)
    {
        Serial.println("Turning RIGHT to search...");
        motor.setSpeed(SEARCH_SPEED, -SEARCH_SPEED);
    }
    else
    {
        Serial.println("Turning LEFT to search...");
        motor.setSpeed(-SEARCH_SPEED, SEARCH_SPEED);
    }

    // Keep spinning until a sensor detects the line
    while (true)
    {
        sensor.readNormalized();

        if (sensor.lineDetected())
        {
            Serial.println("LINE FOUND!");
            pid.reset();   // clear stale integral/derivative before resuming PID
            motor.stop();
            return;
        }

        delay(5);
    }
}