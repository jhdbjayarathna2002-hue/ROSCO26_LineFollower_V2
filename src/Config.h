// ============================================================
// Config.h — ROSCO'26 Task 01 Central Configuration
// ============================================================
// ESP32-C6 | 16-ch Analog MUX | TB6612FNG | PID Line Follower
// All hardware pins, tuning parameters, and feature flags live
// here.  Do NOT scatter magic numbers throughout the code.
// ============================================================

#pragma once

// ------------------------------------------------------------
// SERIAL
// ------------------------------------------------------------
#define SERIAL_BAUD               115200

// ------------------------------------------------------------
// OPERATIONAL MODE
//   true  = DEVELOPMENT MODE  (auto-calibrate on power-on)
//   false = COMPETITION MODE  (load stored calibration,
//                              skip auto-calibrate)
// ------------------------------------------------------------
#define DEVELOPMENT_MODE          true

// ------------------------------------------------------------
// SYSTEM DIAGNOSTIC / TEST MODE SELECTOR
// ------------------------------------------------------------
#define MODE_FULL_AUTONOMOUS      0   // Full state machine & track follower
#define MODE_MOTOR_TEST           1   // Standalone motor test (bypasses sensors & PID)
#define MODE_SENSOR_TEST          2   // Standalone sensor test (motors disabled)
#define MODE_BASIC_PID_TEST       3   // Basic PID line follower (no turn/junction state overrides)

// Select active system mode:
#define SYSTEM_MODE               MODE_FULL_AUTONOMOUS

// ------------------------------------------------------------
// TRACK COLOUR
//   true  = WHITE line on BLACK background  (ROSCO'26 official)
//   false = BLACK line on WHITE background  (lab testing)
// Changing this flag must NOT require changing PID, position,
// motor, or track-detector code.
// ------------------------------------------------------------
#define LINE_IS_WHITE             true

// ------------------------------------------------------------
// SENSOR ARRAY
// ------------------------------------------------------------
#define SENSOR_COUNT              16

// Logical sensor order reversal.
// false = C0 is far LEFT,  C15 is far RIGHT  (default)
// true  = C0 is far RIGHT, C15 is far LEFT
#define SENSOR_REVERSED           false

// ------------------------------------------------------------
// MULTIPLEXER PIN ASSIGNMENT  (16-ch analog MUX)
// E is hardwired to GND — do NOT assign a GPIO to it.
// ------------------------------------------------------------
#define MUX_S0                    0
#define MUX_S1                    1
#define MUX_S2                    2
#define MUX_S3                    3
#define MUX_SIG                   6    // Analog input (ADC)

// ADC resolution used during analogRead
#define ADC_RESOLUTION_BITS       12   // ESP32-C6 ADC = 12-bit (0-4095)

// Settling time (us) after selecting a MUX channel before reading
#define MUX_SETTLE_US             50

// Number of ADC samples averaged per channel per scan
#define SENSOR_AVERAGE_SAMPLES    4

// Optional exponential moving average coefficient (0.0 - 1.0).
// 1.0 = no filtering;  0.7 = 70% new, 30% old
#define SENSOR_EMA_ALPHA          0.8f

// ------------------------------------------------------------
// BUILT-IN RGB LED  (ESP32-C6-DevKitC-1 onboard WS2812)
// Used during calibration to signal "place sensors" and
// "sampling now" without needing a Serial Monitor.
// ------------------------------------------------------------
#define RGB_LED_PIN               8     // GPIO 8 on ESP32-C6-DevKitC-1
#define RGB_LED_BRIGHTNESS        100   // 0-255; raised from 40 so green/red are clearly visible
//   BLUE  (blinking) = waiting — place sensors on surface
//   GREEN (solid)    = sampling in progress
//   OFF              = idle / done / fault

// ------------------------------------------------------------
// CALIBRATION
// ------------------------------------------------------------

// Duration (ms) of the "get ready" countdown blink before sampling
#define CALIBRATION_COUNTDOWN_MS  3000

// Number of samples collected during each calibration phase
#define CALIBRATION_SAMPLES       200

// Inter-sample delay (ms) during calibration
#define CALIBRATION_SAMPLE_DELAY_MS 5

// Minimum raw ADC separation between whiteAverage and blackAverage
// for a sensor to be considered valid.
// If abs(whiteAverage[i] - blackAverage[i]) < this, sensor is INVALID.
#define MIN_CALIBRATION_RANGE     150

// NV key names (for COMPETITION MODE NVS storage)
#define NVS_NAMESPACE             "rosco"
#define NVS_KEY_WHITE             "white"
#define NVS_KEY_BLACK             "black"
#define NVS_KEY_THRESH            "thresh"

// ------------------------------------------------------------
// TB6612FNG PIN ASSIGNMENT
// ------------------------------------------------------------
#define PWMA                      18
#define AIN1                      19
#define AIN2                      20

#define PWMB                      21
#define BIN1                      22
#define BIN2                      23

#define STBY                      7

// ------------------------------------------------------------
// MOTOR INVERSION
// Flip if a motor physically runs backwards.
// ------------------------------------------------------------
#define LEFT_MOTOR_INVERTED       false
#define RIGHT_MOTOR_INVERTED      false

// ------------------------------------------------------------
// PWM (TB6612 PWM channels)
// ------------------------------------------------------------
#define PWM_FREQUENCY             20000   // Hz
#define PWM_RESOLUTION            8       // bits  (0-255)
#define PWM_MAX                   255

// ------------------------------------------------------------
// SPEED PROFILES  (0 - 255)
// ------------------------------------------------------------
#define BASE_SPEED                160
#define STRAIGHT_SPEED            200
#define CURVE_SPEED               140
#define TURN_SPEED                120
#define JUNCTION_SPEED            100
#define SEARCH_SPEED              90

// Smooth speed ramp step per control loop iteration
#define SPEED_RAMP_STEP           3

// ------------------------------------------------------------
// PID PARAMETERS  (starting values - tune on real track)
// ------------------------------------------------------------
#define KP                        0.02f
#define KI                        0.0f
#define KD                        0.08f

// Integral anti-windup clamp  (normalised position units)
#define PID_INTEGRAL_LIMIT        5000.0f

// Derivative smoothing (EMA on error, 1.0 = no smoothing)
#define PID_DERIV_EMA             0.6f

// ------------------------------------------------------------
// LINE DETECTION THRESHOLDS
// ------------------------------------------------------------

// lineStrength value above which a sensor counts as "on line"
#define LINE_THRESHOLD            500

// Below this lineStrength, the reading is treated as noise
#define NOISE_THRESHOLD           80

// Minimum number of sensors above LINE_THRESHOLD to confirm
// a line is detected
#define MIN_ACTIVE_SENSORS        1

// ------------------------------------------------------------
// DASHED LINE / LINE-LOSS HANDLING
// ------------------------------------------------------------

// Max duration (ms) of a short line loss before entering SEARCHING
#define SHORT_LINE_LOSS_MS        150

// Max duration (ms) of SEARCHING before declaring FAULT
#define SEARCH_TIMEOUT_MS         2500

// ------------------------------------------------------------
// 90-DEGREE TURN DETECTION
// ------------------------------------------------------------

// Fraction of total sensors in a group that must be active
// to trigger a hard turn (0.0 - 1.0)
#define TURN_ACTIVATION_RATIO     0.50f

// Sensor group boundaries (0-indexed, inclusive)
// Center group: C7, C8, C9 (3 center sensors)
#define LEFT_GROUP_START          0
#define LEFT_GROUP_END            6
#define CENTER_GROUP_START        7
#define CENTER_GROUP_END          9
#define RIGHT_GROUP_START         10
#define RIGHT_GROUP_END           15

// Minimum active sensors in a wing to detect a junction branch
#define JUNCTION_WING_MIN         3

// Number of consecutive scans a pattern must persist before
// acting on it (temporal filter)
#define PATTERN_PERSIST_COUNT     3

// ------------------------------------------------------------
// CIRCLE / CROSS-CIRCLE DETECTION
// ------------------------------------------------------------

// Minimum active sensors to trigger CIRCLE state.
#define CIRCLE_MIN_SENSORS        10

// How many ms a "wide line" must persist before entering CIRCLE
#define CIRCLE_PERSIST_MS         80

// ------------------------------------------------------------
// DEAD-END DETECTION
// ------------------------------------------------------------

// Dead-end: line appears, then all sensors go dark while
// the robot is still moving forward. Must be significantly longer
// than SHORT_LINE_LOSS_MS (150 ms) to prevent false triggers on dashed lines!
#define DEAD_END_PERSIST_MS       1500

// ------------------------------------------------------------
// JUNCTION NAVIGATION DEFAULT
//   0 = STRAIGHT, 1 = LEFT, 2 = RIGHT
// ------------------------------------------------------------
#define JUNCTION_DEFAULT_DIRECTION  0

// ------------------------------------------------------------
// CONTROL LOOP TIMING
// ------------------------------------------------------------

// Minimum interval (us) between full sensor+PID+motor cycles
#define CONTROL_LOOP_US           2000   // 500 Hz max

// Debug serial print interval (ms)
#define DEBUG_PRINT_MS            100

// Sensor-test mode loop delay (ms)
#define SENSOR_TEST_DELAY_MS      50

// ------------------------------------------------------------
// DEBUG FLAGS  (uncomment to enable)
// ------------------------------------------------------------
// #define DEBUG_SENSOR
// #define DEBUG_PID
// #define DEBUG_MOTOR
// #define DEBUG_TRACK
// #define DEBUG_PATTERN
// #define DEBUG_CALIBRATION
#define DEBUG_STATE_CHANGE
#define DEBUG_MOTOR_COMMANDS

// ============================================================
// END Config.h
// ============================================================
