#pragma once

// ============================================================
// ROSCO'26 TASK 01
// ESP32-C6 + 16-CHANNEL IR MUX + TB6612FNG
//
// TRACK:
// BLACK LINE
// WHITE BACKGROUND
// ============================================================


// ============================================================
// SERIAL
// ============================================================

#define SERIAL_BAUD 115200


// ============================================================
// IR SENSOR MUX
// ============================================================

#define SENSOR_COUNT 16

#define MUX_S0 0
#define MUX_S1 1
#define MUX_S2 2
#define MUX_S3 3

#define MUX_SIG 6

// MUX EN/E is connected directly to GND

#define ADC_RESOLUTION 12

#define MUX_SETTLE_US 60

#define SENSOR_AVERAGE_SAMPLES 3


// ============================================================
// TRACK COLOUR
// ============================================================
//
// Your actual track:
//
// WHITE BACKGROUND
// BLACK LINE
//
// Therefore:
//
// BLACK = LINE
// WHITE = BACKGROUND
//
// true  = inverse / black-line following
// false = normal / white-line following
// ============================================================

#define INVERT_LINE_FOLLOWING true


// ============================================================
// SENSOR ORDER
// ============================================================
//
// C0  = LEFT
// C15 = RIGHT
// ============================================================

#define SENSOR_REVERSED false


// ============================================================
// RGB LED
// ============================================================

#define RGB_LED_PIN 8

#define RGB_LED_BRIGHTNESS 80


// ============================================================
// INDIVIDUAL CALIBRATION
// ============================================================
//
// Each sensor gets its OWN:
//
// C0  → black + white
// C1  → black + white
// ...
// C15 → black + white
//
// No common ADC threshold.
//
// Calibration happens ONLY at startup.
// ============================================================

#define CALIBRATION_SAMPLES 200

#define CALIBRATION_SAMPLE_DELAY_MS 2

#define CALIBRATION_COUNTDOWN_MS 3000

#define MIN_CALIBRATION_RANGE 150


// ============================================================
// TB6612FNG
// ============================================================

#define PWMA 18
#define AIN1 19
#define AIN2 20

#define PWMB 21
#define BIN1 22
#define BIN2 23

#define STBY 7


// ============================================================
// MOTOR DIRECTION
// ============================================================
//
// Change these ONLY if a motor physically runs
// in the wrong direction.
// ============================================================

#define LEFT_MOTOR_INVERTED false
#define RIGHT_MOTOR_INVERTED false


// ============================================================
// MOTOR PWM
// ============================================================

#define PWM_FREQUENCY 20000

#define PWM_RESOLUTION 8

#define PWM_MAX 255


// ============================================================
// LINE FOLLOWING SPEED
// ============================================================
//
// Start relatively slow.
// We will tune speed after basic following works.
// ============================================================

#define BASE_SPEED 130

#define MAX_SPEED 255

#define SEARCH_SPEED 100

#define MOTOR_MIN_PWM 75


// ============================================================
// PID
// ============================================================
//
// Starting values only.
// ============================================================

#define KP 0.01f

#define KI 0.0000f

#define KD 0.055f

#define PID_INTEGRAL_LIMIT 4000.0f


// ============================================================
// LINE DETECTION
// ============================================================
//
// Normalized range:
//
// BLACK LINE in inverse mode = 1000
// WHITE background            = 0
// ============================================================

#define LINE_STRENGTH_THRESHOLD 500

#define MIN_LINE_STRENGTH_SUM 250


// ============================================================
// CONTROL LOOP
// ============================================================

#define CONTROL_LOOP_US 2500


// ============================================================
// LINE LOSS / DASHED LINE
// ============================================================
//
// Rulebook allows dashed gaps.
// These values will be tuned later.
// ============================================================

#define DASHED_LINE_TIME_MS 180

#define SEARCH_TIMEOUT_MS 1000


// ============================================================
// 90 DEGREE TURN
// ============================================================
//
// Not actively used in our first basic test.
// Kept ready for the next stage.
// ============================================================

#define TURN_SIDE_COUNT 3

#define TURN_CENTER_MAX 1

#define TURN_TIMEOUT_MS 700


// ============================================================
// JUNCTION
// ============================================================
//
// Not used in the first basic line follower.
// ============================================================

#define JUNCTION_SIDE_COUNT 3

#define JUNCTION_CONFIRM_MS 30


// ============================================================
// CIRCLE
// ============================================================

#define CIRCLE_ACTIVE_COUNT 10

#define CIRCLE_CONFIRM_MS 70


// ============================================================
// DEBUG
// ============================================================
//
// Uncomment only when required.
//
// #define DEBUG_SENSOR
// #define DEBUG_POSITION
// #define DEBUG_PID