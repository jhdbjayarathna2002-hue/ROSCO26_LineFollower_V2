#pragma once

// ============================================================
// ROSCO'26 LINE FOLLOWER
// ESP32-C6 + 16 IR MUX + TB6612FNG
// ============================================================


// ============================================================
// SERIAL
// ============================================================

#define SERIAL_BAUD 115200


// ============================================================
// TRACK COLOUR
// ============================================================
//
// CURRENT TEST TRACK:
// BLACK LINE
// WHITE BACKGROUND
//
// true  = black line
// false = white line
//
// For official ROSCO track later:
// set this to false.
// ============================================================

#define INVERT_LINE_FOLLOWING false


// ============================================================
// IR SENSOR MUX
// ============================================================

#define SENSOR_COUNT 16

#define MUX_S0 0
#define MUX_S1 1
#define MUX_S2 2
#define MUX_S3 3

#define MUX_SIG 6

// MUX EN/E is connected directly to GND.

#define ADC_RESOLUTION 12

#define MUX_SETTLE_US 60

#define SENSOR_AVERAGE_SAMPLES 3


// ============================================================
// SENSOR ORDER
// ============================================================
//
// Physical order:
//
// LEFT
//
// C0 C1 C2 C3 C4 C5 C6 C7 C8 C9 C10 C11 C12 C13 C14 C15
//
// RIGHT
//
// C7 + C8 = CENTER PAIR
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
// Each sensor gets its own:
//
// BLACK value
// WHITE value
// THRESHOLD
//
// Calibration happens ONLY once during startup.
// ============================================================

#define CALIBRATION_SAMPLES 200

#define CALIBRATION_SAMPLE_DELAY_MS 2

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
// Leave these as they are while the motors are working
// correctly.
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
// MOTOR SPEED
// ============================================================

#define BASE_SPEED 150

#define STRAIGHT_SPEED 140

#define CURVE_SPEED 100

#define TURN_SPEED 115
#define SEARCH_SPEED 110
#define REVERSE_SPEED 75

#define JUNCTION_SPEED 100
#define CIRCLE_SPEED 100


// ============================================================
// MOTOR STARTING PWM
// ============================================================
//
// Prevents motors from receiving a PWM value that is too
// small to start the TT motors reliably.
// ============================================================

#define MOTOR_MIN_PWM 75


// ============================================================
// PID
// ============================================================

#define KP 0.01f

#define KI 0.0000f

#define KD 0.055f

#define PID_INTEGRAL_LIMIT 4000.0f


// ============================================================
// SENSOR LINE DETECTION
// ============================================================

#define LINE_STRENGTH_THRESHOLD 500

#define MIN_LINE_STRENGTH_SUM 250


// ============================================================
// SENSOR POSITION
// ============================================================
//
// C7 = -500
// C8 = +500
//
// Therefore the center of the robot is approximately 0.
// ============================================================

#define CENTER_POSITION 0


// ============================================================
// CONTROL LOOP
// ============================================================

#define CONTROL_LOOP_US 2500


// ============================================================
// TASK 01 — CURVE
// ============================================================

#define CURVE_POSITION_LIMIT 2500


// ============================================================
// TASK 01 — 90° TURN
// ============================================================

#define TURN_MIN_WING_SENSORS 3

#define TURN_MIN_CENTER_SENSORS 1

#define TURN_REACQUIRE_SENSORS 2

#define TURN_MIN_TIME_MS 100

#define TURN_TIMEOUT_MS 700


// ============================================================
// TASK 01 — DASHED LINE
// ============================================================
//
// Initial value only.
// We will tune this on the actual track.
// ============================================================

#define DASHED_LOSS_MS 180

#define SEARCH_TIMEOUT_MS 1200


// ============================================================
// TASK 01 — JUNCTION
// ============================================================

#define JUNCTION_MIN_WING_SENSORS 3

#define JUNCTION_CONFIRM_MS 30


// ============================================================
// TASK 01 — CIRCLE
// ============================================================

#define CIRCLE_MIN_ACTIVE 10

#define CIRCLE_CONFIRM_MS 70

#define CIRCLE_EXIT_ACTIVE 6


// ============================================================
// TASK 01 — DEAD END
// ============================================================

#define DEAD_END_TIMEOUT_MS 900


// ============================================================
// DEBUG
// ============================================================
//
// Uncomment only when needed.
//
// #define DEBUG_SENSOR
// #define DEBUG_POSITION
// #define DEBUG_PID
// #define DEBUG_TRACK