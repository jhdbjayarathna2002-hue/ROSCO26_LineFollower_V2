#pragma once

#include <Arduino.h>
#include "Config.h"

// ============================================================
// ROSCO'26 TASK 01
// TRACK LOGIC
// ============================================================
//
// Handles:
//
//  1. Normal line following (PID)
//  2. Curves
//  3. 90-degree left turns
//  4. 90-degree right turns
//  5. Short line gaps / dashed sections
//  6. Line search / recovery
//  7. Dead-end recovery
//  8. L/T/cross junctions
//  9. Circular / cross-circle sections
// 10. Task 01 checkpoint completion
//
// Sensor processing -> Sensors
// PID calculation   -> PID
// Motor control     -> Motor
// Track behaviour   -> TrackLogic
// ============================================================


// ============================================================
// TRACK STATES
// ============================================================

enum TrackState
{
    TRACK_NORMAL,

    TRACK_CURVE,

    TRACK_TURN_LEFT,

    TRACK_TURN_RIGHT,

    TRACK_GAP,

    TRACK_SEARCH,

    TRACK_DEAD_END,

    TRACK_JUNCTION,

    TRACK_CIRCLE,

    TRACK_TASK01_COMPLETE
};


// ============================================================
// TRACK LOGIC CLASS
// ============================================================

class TrackLogic
{
public:

    // --------------------------------------------------------
    // Initialize
    // --------------------------------------------------------

    void begin();


    // --------------------------------------------------------
    // Main update
    // --------------------------------------------------------

    void update();


    // --------------------------------------------------------
    // Get current state
    // --------------------------------------------------------

    TrackState state();


    // --------------------------------------------------------
    // Reset to normal following
    // --------------------------------------------------------

    void reset();


    // --------------------------------------------------------
    // Print current state
    // --------------------------------------------------------

    void printState();


private:

    // ========================================================
    // STATE HANDLERS
    // ========================================================

    void handleNormal();

    void handleCurve();

    void handleTurnLeft();

    void handleTurnRight();

    void handleGap();

    void handleSearch();

    void handleDeadEnd();

    void handleJunction();

    void handleCircle();

    void handleTask01Complete();


    // ========================================================
    // FEATURE DETECTION
    // ========================================================

    bool detectLeftTurn();

    bool detectRightTurn();

    bool detectCurve();

    bool detectLineLoss();

    bool detectJunction();

    bool detectCircle();

    bool detectCircleExit();


    // ========================================================
    // TURN CONTROL
    // ========================================================

    void startLeftTurn();

    void startRightTurn();

    bool turnReacquired();


    // ========================================================
    // STATE CHANGE
    // ========================================================

    void changeState(
        TrackState newState
    );


    // ========================================================
    // SENSOR GROUPS
    // ========================================================

    int leftActive();

    int centerActive();

    int rightActive();


    // ========================================================
    // TIMERS
    // ========================================================

    unsigned long stateStartTime;

    unsigned long lineLostTime;

    // Turn candidate persistence guard (TURN_MIN_TIME_MS).
    unsigned long turnCandidateTime;
    // 0 = none, -1 = left, +1 = right
    int turnCandidateDir;

    // Curve candidate persistence guard (CURVE_CONFIRM_MS).
    unsigned long curveCandidateTime;
    bool curveCandidateActive;

    // Junction candidate persistence guard (JUNCTION_CONFIRM_MS).
    unsigned long junctionCandidateTime;
    bool junctionCandidateActive;

    // Circle candidate persistence guard (CIRCLE_CONFIRM_MS / CIRCLE_FOLLOW_EXIT_MS).
    unsigned long circleCandidateTime;
    bool circleCandidateActive;

    // Rate-limit for DEBUG_TRACK continuous output.
    unsigned long lastDebugTime;


    // ========================================================
    // LAST KNOWN POSITION
    // ========================================================

    int lastLinePosition;


    // ========================================================
    // CURRENT STATE
    // ========================================================

    TrackState currentState;
};


// ============================================================
// GLOBAL OBJECT
// ============================================================

extern TrackLogic track;