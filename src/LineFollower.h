// ============================================================
// LineFollower.h  —  High-level robot state machine
// ============================================================
// Orchestrates:
//   - Sensor updates
//   - Track detection
//   - PID control
//   - Motor commands
//   - Speed profiles
//   - Line-loss / dashed-line handling
//   - Search routine
//   - 90-degree turns (sensor-guided)
//   - Junction decisions
//   - Circle / cross-circle following
//   - Dead-end response
//   - Fault handling
//
// State machine:
//   STARTUP -> CALIBRATION -> READY -> RUNNING
//   RUNNING sub-states mirror TrackState (via TrackDetector)
// ============================================================

#pragma once
#include <Arduino.h>
#include "Config.h"
#include "Sensor.h"
#include "Motor.h"
#include "PID.h"
#include "TrackDetector.h"

// ---- Top-level robot state ----------------------------------
enum RobotState : uint8_t
{
    ROBOT_STARTUP = 0,
    ROBOT_CALIBRATING,
    ROBOT_READY,
    ROBOT_RUNNING,
    ROBOT_FAULT
};

class LineFollower
{
public:
    LineFollower();

    // ---- Lifecycle -------------------------------------------

    // Initialise all subsystems.  Call once from setup().
    void begin();

    // Run one complete control iteration.
    // Call from loop() — must be fast (no blocking delays).
    void update();

    // ---- Accessors -------------------------------------------
    RobotState   getRobotState() const { return _robotState; }
    TrackState   getTrackState() const { return detector.getState(); }

    // Public subsystems (accessible for debug / testing)
    Sensor       sensor;
    Motor        motor;
    PID          pid;
    TrackDetector detector;

private:
    // ---- Robot state machine ---------------------------------
    RobotState  _robotState;

    // ---- Timing ----------------------------------------------
    unsigned long _lastLoopUs;       // micros of last control iteration
    unsigned long _lastDebugMs;      // millis of last debug print
    unsigned long _lineLostStartMs;  // when line loss began
    unsigned long _searchStartMs;    // when SEARCHING began

    // ---- Speed profile state ---------------------------------
    int  _targetSpeed;               // desired base speed
    int  _currentSpeed;              // ramp-tracked current speed

    // ---- Turn state -----------------------------------------
    bool _inTurn;                    // true during a 90-deg turn
    int  _turnDirection;             // -1 = left, +1 = right

    // ---- Junction state -------------------------------------
    bool _inJunction;

    // ---- Short line-loss (dashed line) ----------------------
    bool _shortLineLoss;

    // ---- Internal methods -----------------------------------

    // Run startup development calibration (called from begin())
    void runDevelopmentCalibration();

    // Run competition mode load (called from begin())
    void runCompetitionCalibration();

    // Main control loop body (called from update())
    void runControl(float dt);

    // ---- Diagnostic Test Modes -------------------------------
    void runMotorTest();
    void runSensorTest();
    void runBasicPidTest(float dt);

    // ---- State handlers -------------------------------------
    void handleNormalLine(float dt);
    void handleCurve(float dt);
    void handleTurn(float dt);
    void handleJunction(float dt);
    void handleCircle(float dt);
    void handleLineLost(float dt);
    void handleSearching(float dt);
    void handleDeadEnd(float dt);
    void handleFault();

    // ---- Helpers --------------------------------------------
    void applyMotors(int baseSpeed, float correction);
    void rampSpeed(int target);
    void enterFault(const char* reason);
    void printDebug();
};
