// ============================================================
// ROSCO'26 TASK 01
// MODULAR LINE FOLLOWER
//
// ESP32-C6
// 16 IR sensors through MUX
// TB6612FNG motor driver
//
// Current development track:
// BLACK LINE
// WHITE BACKGROUND
//
// Sensor center:
// C7 + C8
// ============================================================

#include <Arduino.h>

#include "Config.h"
#include "Sensors.h"
#include "Motor.h"
#include "PID.h"
#include "TrackLogic.h"


// ============================================================
// SETUP
// ============================================================

void setup()
{
    // --------------------------------------------------------
    // Serial
    // --------------------------------------------------------

    Serial.begin(
        SERIAL_BAUD
    );

    delay(1000);


    Serial.println();
    Serial.println();
    Serial.println(
        "========================================"
    );

    Serial.println(
        "      ROSCO'26 TASK 01"
    );

    Serial.println(
        "      MODULAR LINE FOLLOWER"
    );

    Serial.println(
        "========================================"
    );

    Serial.println();


    // --------------------------------------------------------
    // Initialize sensors
    // --------------------------------------------------------

    sensors.begin();


    // --------------------------------------------------------
    // Initialize motors
    // --------------------------------------------------------

    motor.begin();


    // --------------------------------------------------------
    // Initialize PID
    // --------------------------------------------------------

    pid.begin();


    // --------------------------------------------------------
    // Motor driver remains disabled during calibration.
    // --------------------------------------------------------

    motor.disable();


    // --------------------------------------------------------
    // Calibration
    // --------------------------------------------------------
    //
    // WHITE
    //    ↓
    // BLACK
    //    ↓
    // Individual thresholds
    //    ↓
    // LOCK
    //
    // Motors are NOT controlled during calibration.
    // --------------------------------------------------------

    bool calibrationOK =
        sensors.calibrate();


    // --------------------------------------------------------
    // Calibration failed
    // --------------------------------------------------------

    if (
        !calibrationOK
    )
    {
        Serial.println();

        Serial.println(
            "========================================"
        );

        Serial.println(
            "CALIBRATION FAILED"
        );

        Serial.println(
            "ROBOT WILL NOT MOVE"
        );

        Serial.println(
            "CHECK SENSOR VALUES"
        );

        Serial.println(
            "========================================"
        );


        motor.emergencyStop();


        while (true)
        {
            delay(1000);
        }
    }


    // --------------------------------------------------------
    // Calibration successful
    // --------------------------------------------------------

    Serial.println();

    Serial.println(
        "========================================"
    );

    Serial.println(
        "CALIBRATION SUCCESSFUL"
    );

    Serial.println(
        "========================================"
    );


    // --------------------------------------------------------
    // Initialize track logic
    // --------------------------------------------------------

    track.begin();


    // --------------------------------------------------------
    // Small delay before motors start
    // --------------------------------------------------------

    Serial.println();

    Serial.println(
        "Robot starting in 2 seconds..."
    );

    delay(2000);


    // --------------------------------------------------------
    // Enable motor driver
    // --------------------------------------------------------

    motor.enable();


    Serial.println();

    Serial.println(
        "========================================"
    );

    Serial.println(
        "TASK 01 LINE FOLLOWING STARTED"
    );

    Serial.println(
        "========================================"
    );
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    // --------------------------------------------------------
    // Read all 16 IR sensors
    // --------------------------------------------------------

    sensors.update();


    // --------------------------------------------------------
    // Run track logic
    // --------------------------------------------------------

    track.update();
}