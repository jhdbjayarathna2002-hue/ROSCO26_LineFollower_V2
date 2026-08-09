#pragma once

#include <Arduino.h>
#include "Config.h"

// ============================================================
// TRACK LOGIC MODULE
// ============================================================
//
// This module controls the robot's behaviour on Task 01.
//
// Current planned behaviours:
//
// 1. NORMAL LINE FOLLOWING
// 2. CURVE
// 3. 90° LEFT TURN
// 4. 90° RIGHT TURN
// 5. DASHED LINE / SHORT GAP
// 6. JUNCTION
// 7. CIRCLE / CROSS-CIRCLE
// 8. LINE SEARCH
// 9. DEAD-END RECOVERY
//
// Sensor processing is handled by Sensors.cpp.
// PID calculation is handled by PID.cpp.
// Motor control is handled by Motor.cpp.
//
// TrackLogic only decides:
// "What should the robot do now?"
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

    TRACK_DASHED,

    TRACK_JUNCTION,

    TRACK_CIRCLE,

    TRACK_SEARCH,

    TRACK_DEAD_END
};


// ============================================================
// TRACK LOGIC CLASS
// ============================================================

class TrackLogic
{
public:

    // --------------------------------------------------------
    // Initialize track logic
    // --------------------------------------------------------

    void begin();


    // --------------------------------------------------------
    // Main track-control update
    //
    // Called repeatedly from main.cpp.
    // --------------------------------------------------------

    void update();


    // --------------------------------------------------------
    // Current state
    // --------------------------------------------------------

    TrackState state();


    // --------------------------------------------------------
    // Force normal line following
    // --------------------------------------------------------

    void reset();


    // --------------------------------------------------------
    // Debug
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

    void handleDashed();

    void handleJunction();

    void handleCircle();

    void handleSearch();

    void handleDeadEnd();


    // ========================================================
    // DETECTION
    // ========================================================

    void detectTrackFeature();


    bool detectLeftTurn();

    bool detectRightTurn();

    bool detectJunction();

    bool detectCircle();

    bool detectLineLoss();


    // ========================================================
    // TURN CONTROL
    // ========================================================

    void startLeftTurn();

    void startRightTurn();

    bool turnLineReacquired();


    // ========================================================
    // STATE MANAGEMENT
    // ========================================================

    void changeState(
        TrackState newState
    );


    // ========================================================
    // SENSOR GROUP COUNTS
    // ========================================================

    int leftActive();

    int centerActive();

    int rightActive();


    // ========================================================
    // TIMERS
    // ========================================================

    unsigned long stateStartTime;

    unsigned long lineLostTime;


    // ========================================================
    // LAST KNOWN LINE
    // ========================================================

    int lastLinePosition;


    // ========================================================
    // TURN DIRECTION
    //
    // -1 = left
    // +1 = right
    // ========================================================

    int turnDirection;


    // ========================================================
    // CURRENT STATE
    // ========================================================

    TrackState currentState;
};


// ============================================================
// GLOBAL TRACK LOGIC OBJECT
// ============================================================

extern TrackLogic track;