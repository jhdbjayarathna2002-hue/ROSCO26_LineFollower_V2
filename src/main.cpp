// ============================================================
// main.cpp  —  ROSCO'26 Task 01 Entry Point
// ============================================================
// Startup sequence:
//   setup()  -> LineFollower::begin()
//              -> Sensor::begin()
//              -> Motor::begin()    (STBY LOW)
//              -> calibrate()       (STBY LOW during calibration)
//              -> PID::reset()
//              -> Motor::enable()   (STBY HIGH)
//   loop()   -> LineFollower::update()  (fast, non-blocking)
//
// calibrate() is called ONLY from setup(), NEVER from loop().
//
// Debug: uncomment desired DEBUG_* flags in Config.h.
//        Serial output is rate-limited and does NOT block
//        the motor control loop.
// ============================================================

#include <Arduino.h>
#include "Config.h"
#include "LineFollower.h"

LineFollower robot;

// ============================================================
// setup()
// ============================================================

void setup()
{
    Serial.begin(SERIAL_BAUD);
    delay(500);   // Allow Serial to settle

    // begin() performs:
    //   1. Sensor GPIO + ADC initialisation
    //   2. Motor GPIO + PWM initialisation  (STBY LOW)
    //   3. Calibration (auto-cal in DEVELOPMENT_MODE,
    //                   load from NVS in COMPETITION_MODE)
    //   4. PID reset
    //   5. Motor enable (STBY HIGH) if calibration OK
    robot.begin();

    // If calibration failed, robot is in ROBOT_FAULT — loop() is safe.
}

// ============================================================
// loop()  —  non-blocking, called as fast as possible
// ============================================================

void loop()
{
    // All timing, state management, and motor control happens
    // inside LineFollower::update().  loop() does nothing else
    // to avoid adding latency.
    robot.update();
}