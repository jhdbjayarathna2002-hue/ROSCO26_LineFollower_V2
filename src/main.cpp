// ============================================================
// ROSCO'26 ROBOT CONTROLLER
//
// TASK 1: Existing 16-sensor line follower
// TASK 2: Temporary four-room wall-following test
//
// ESP32-C6 + TB6612FNG + TCA9548A + 3 x VL53L0X
// ============================================================

#include <Arduino.h>

#include "Config.h"
#include "Sensors.h"
#include "Motor.h"
#include "PID.h"
#include "TrackLogic.h"
#include "ToFSensors.h"
#include "Task2.h"

enum RobotTaskMode
{
    ROBOT_TASK_1,
    ROBOT_TASK_2_TEST
};

static RobotTaskMode currentTask = ROBOT_TASK_1;

static bool lastButtonReading = HIGH;
static bool stableButtonState = HIGH;
static unsigned long lastButtonChangeTime = 0;

static void blinkPinkLED()
{
    for (int i = 0; i < 3; i++)
    {
        rgbLedWrite(RGB_LED_PIN, RGB_LED_BRIGHTNESS, 0, RGB_LED_BRIGHTNESS);
        delay(100);
        rgbLedWrite(RGB_LED_PIN, 0, 0, 0);
        delay(100);
    }
}

static void startTask1()
{
    task2.stop();
    track.reset();
    pid.reset();
    motor.enable();

    currentTask = ROBOT_TASK_1;
    blinkPinkLED();

    Serial.println();
    Serial.println("========================================");
    Serial.println("TASK 01 LINE FOLLOWING STARTED");
    Serial.println("========================================");
}

static void startTask2Test()
{
    motor.stop();
    track.reset();
    pid.reset();

    currentTask = ROBOT_TASK_2_TEST;
    blinkPinkLED();

    Serial.println();
    Serial.println("========================================");
    Serial.println("TASK 02 WALL-FOLLOWING TEST STARTED");
    Serial.println("START POSITION: ON LINE BEFORE JUNCTION 1");
    Serial.println("========================================");

    task2.start();
}

static void processTaskButton()
{
    bool reading = digitalRead(TASK_BUTTON_PIN);

    if (reading != lastButtonReading)
    {
        lastButtonChangeTime = millis();
        lastButtonReading = reading;
    }

    if (millis() - lastButtonChangeTime <
        (unsigned long)TASK_BUTTON_DEBOUNCE_MS)
    {
        return;
    }

    if (reading != stableButtonState)
    {
        stableButtonState = reading;

        // Button is active LOW because it is connected to GND.
        if (stableButtonState == LOW)
        {
            motor.stop();

            if (currentTask == ROBOT_TASK_1)
            {
                startTask2Test();
            }
            else
            {
                startTask1();
            }
        }
    }
}

void setup()
{
    pinMode(TASK_BUTTON_PIN, INPUT_PULLUP);
    delay(20);

    // Latch this immediately, so the button does not need to remain held
    // during the line-sensor calibration sequence.
    bool startDirectlyInTask2 = digitalRead(TASK_BUTTON_PIN) == LOW;

    Serial.begin(SERIAL_BAUD);
    delay(1000);

    Serial.println();
    Serial.println("========================================");
    Serial.println("ROSCO'26 ROBOT CONTROLLER");
    Serial.println("TASK 1 + TASK 2 WALL TEST");
    Serial.println("========================================");

    sensors.begin();
    motor.begin();
    pid.begin();
    task2.begin();

    motor.disable();

    bool calibrationOK = sensors.calibrate();

    if (!calibrationOK)
    {
        Serial.println();
        Serial.println("========================================");
        Serial.println("LINE SENSOR CALIBRATION FAILED");
        Serial.println("ROBOT WILL NOT MOVE");
        Serial.println("========================================");

        motor.emergencyStop();

        while (true)
        {
            delay(1000);
        }
    }

    Serial.println("[MAIN] Line sensor calibration successful");

    // Task 1 still works if ToF initialisation fails. Task 2 will remain
    // stopped and will print an error if selected.
    bool tofOK = tofSensors.begin();

    if (!tofOK)
    {
        Serial.println("[MAIN] Task 2 unavailable; check TCA/ToF wiring");
    }

    track.begin();

    Serial.println();
    Serial.println("Robot starting in 2 seconds...");
    delay(2000);

    lastButtonReading = digitalRead(TASK_BUTTON_PIN);
    stableButtonState = lastButtonReading;
    lastButtonChangeTime = millis();

    if (startDirectlyInTask2)
    {
        startTask2Test();
    }
    else
    {
        startTask1();
    }
}

void loop()
{
    processTaskButton();

    // Required by Task 1 and by Task 2 junction detection.
    sensors.update();

    if (currentTask == ROBOT_TASK_1)
    {
        track.update();
    }
    else
    {
        task2.update();
    }
}
