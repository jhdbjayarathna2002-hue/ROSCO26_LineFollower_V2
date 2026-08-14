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
// POSITION SIGN
// ============================================================
//
// If the robot steers AWAY from the line instead of toward it,
// change this from +1 to -1. This flips the PID correction
// direction without touching any logic code.
// ============================================================

#define POSITION_SIGN (-1)


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

#define BASE_SPEED 240

#define STRAIGHT_SPEED 190

#define CURVE_SPEED 150

#define TURN_SPEED 160
#define SEARCH_SPEED 130
#define REVERSE_SPEED 140

#define JUNCTION_SPEED 140
#define CIRCLE_SPEED 140


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

#define KP 0.04f

#define KI 0.0000f

// KD keeps overshoot under control at the higher speed.
#define KD 0.000f

#define PID_INTEGRAL_LIMIT 4000.0f


// ============================================================
// SENSOR LINE DETECTION
// ============================================================

#define LINE_STRENGTH_THRESHOLD 500

// Minimum total strength across all sensors to declare line detected.
// 350 = safely above noise, well below a single sensor fully on the line (~1000).
#define MIN_LINE_STRENGTH_SUM 350


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

// Raised from 2500 -> 6000.
// With sensor weights -7500..+7500, the old value of 2500 triggered
// CURVE on any slight off-center position on a straight section.
// 6000 only fires for genuinely tight curves.
#define CURVE_POSITION_LIMIT 6000

// Pattern must persist this long before entering CURVE state.
// Prevents one noisy reading from triggering the state change.
#define CURVE_CONFIRM_MS 60


// ============================================================
// TASK 01 — 90° TURN
// ============================================================

// Raised from 3 -> 4 to prevent false turn triggers when robot drifts.
#define TURN_MIN_WING_SENSORS 4

#define TURN_MIN_CENTER_SENSORS 1

#define TURN_REACQUIRE_SENSORS 2

// Raised from 100 -> 200 ms for stronger turn confirmation.
#define TURN_MIN_TIME_MS 200

#define TURN_TIMEOUT_MS 700


// ============================================================
// TASK 01 — DASHED LINE
// ============================================================
//
// Initial value only.
// We will tune this on the actual track.
// ============================================================

// At STRAIGHT_SPEED=160, 450 ms covers enough distance to cross
// the 50mm dashed gap, but triggers the 180-turn fast at a dead end.
#define DASHED_LOSS_MS 450

#define SEARCH_TIMEOUT_MS 3000


// ============================================================
// TASK 01 — JUNCTION
// ============================================================

#define JUNCTION_MIN_WING_SENSORS 4

// Center sensors required for junction (distinguishes from pure turn).
#define JUNCTION_MIN_CENTER_SENSORS 2

// Total active sensor count required to declare junction.
#define JUNCTION_TOTAL_MIN 8

#define JUNCTION_CONFIRM_MS 100

// Junction navigation strategy:
//   0 = STRAIGHT (continue through on center line)
//   1 = LEFT     (take the left branch)
//   2 = RIGHT    (take the right branch)
#define JUNCTION_NAV_STRAIGHT  0
#define JUNCTION_NAV_LEFT      1
#define JUNCTION_NAV_RIGHT     2
#define JUNCTION_NAV_DEFAULT   JUNCTION_NAV_STRAIGHT


// ============================================================
// TASK 01 — CIRCLE
// ============================================================

#define CIRCLE_MIN_ACTIVE 13

#define CIRCLE_CONFIRM_MS 150

#define CIRCLE_EXIT_ACTIVE 6

// Active-count must remain below CIRCLE_EXIT_ACTIVE for this long
// before the circle exit is confirmed (avoids premature exit).
#define CIRCLE_FOLLOW_EXIT_MS 200


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
#define DEBUG_TRACK

// Rate-limit for DEBUG_TRACK continuous output (ms).
// Keeps Serial from flooding the control loop.
#define DEBUG_PRINT_RATE_MS 150


// ============================================================
// TASK SELECT BUTTON
// ============================================================
// Connect a normally-open push button between GPIO10 and GND.
// The ESP32 internal pull-up is used, so no external resistor is needed.
// Hold the button while powering/resetting the robot to start Task 2 test.
// A short press while running toggles between Task 1 and Task 2 test.

#define TASK_BUTTON_PIN 10
#define TASK_BUTTON_DEBOUNCE_MS 50


// ============================================================
// TASK 02 — TCA9548A + THREE VL53L0X SENSORS
// ============================================================

#define TOF_SDA_PIN 4
#define TOF_SCL_PIN 5

#define TCA9548A_ADDRESS 0x70

#define TOF_LEFT_CHANNEL 0
#define TOF_FRONT_CHANNEL 1
#define TOF_RIGHT_CHANNEL 2

#define TOF_DEFAULT_ADDRESS 0x29

#define TOF_LEFT_OFFSET_MM 0
#define TOF_FRONT_OFFSET_MM 0
#define TOF_RIGHT_OFFSET_MM 0

// Exponential filter: 0.0 = very slow, 1.0 = no filtering.
#define TOF_FILTER_ALPHA 0.35f


// ============================================================
// TASK 02 — ROOM CONTROL
// ============================================================
// Room width = 300 mm, robot width = 210 mm.
// Nominal side clearance when centered = (300 - 210) / 2 = 45 mm.
// The controller primarily uses LEFT - RIGHT, so exact sensor mounting
// offsets do not affect centering when both sensors are mounted symmetrically.

#define TASK2_ROOM_WIDTH_MM 300
#define TASK2_ROBOT_WIDTH_MM 210
#define TASK2_NOMINAL_SIDE_CLEARANCE_MM 45

// Stop this far from the front wall or box. Tune on the real robot.
#define TASK2_FRONT_STOP_MM 70

// Front distance expected near the room entrance while reversing out.
#define TASK2_ROOM_EXIT_FRONT_MM 300

#define TASK2_BASE_SPEED 120
#define TASK2_REVERSE_SPEED 115
#define TASK2_TURN_SPEED 130
#define TASK2_LINE_SPEED 130

#define TASK2_WALL_KP 1.20f
#define TASK2_WALL_KD 0.60f
#define TASK2_MAX_CORRECTION 45

// Time-based turns must be calibrated on the physical robot.
#define TASK2_TURN_90_MS 500
#define TASK2_TURN_180_MS 1000
#define TASK2_TURN_SETTLE_MS 200

#define TASK2_REVERSE_MIN_MS 650
#define TASK2_REVERSE_TIMEOUT_MS 3000

// Ignore the junction under the robot immediately after turning onto the
// central line, then look for the next wide junction pattern.
#define TASK2_JUNCTION_IGNORE_MS 700
#define TASK2_JUNCTION_CONFIRM_MS 100

#define TASK2_TOF_UPDATE_MS 60
#define TASK2_DEBUG_RATE_MS 200
