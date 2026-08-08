// ============================================================
// TrackDetector.h  —  Track element classification
// ============================================================
// Classifies the current track section by analysing the
// 16-bit sensor pattern, lineStrength[], position, and
// temporal history.
//
// States:
//   NORMAL_LINE   Plain straight or gently curving line
//   CURVE         Significant curve; moderate sensor shift
//   TURN_LEFT     Hard 90-degree left turn in progress
//   TURN_RIGHT    Hard 90-degree right turn in progress
//   JUNCTION      T/L/cross junction detected
//   CIRCLE        Circular path (wide sensor activation)
//   DASHED_LINE   Short gap in line (dashed section)
//   LINE_LOST     Line not detected, awaiting recovery
//   SEARCHING     Active search manoeuvre
//   RECOVERY      Line reacquired, PID recovering
//   DEAD_END      Forward motion leads to permanent line loss
//   FAULT         Unrecoverable error
//
// Only lineStrength[] and pattern are consumed here.
// The detector does NOT directly command motors.
// ============================================================

#pragma once
#include <Arduino.h>
#include "Config.h"
#include "Sensor.h"

// ---- Track state enum ---------------------------------------
enum TrackState : uint8_t
{
    STATE_NORMAL_LINE = 0,
    STATE_CURVE,
    STATE_TURN_LEFT,
    STATE_TURN_RIGHT,
    STATE_JUNCTION,
    STATE_CIRCLE,
    STATE_DASHED_LINE,
    STATE_LINE_LOST,
    STATE_SEARCHING,
    STATE_RECOVERY,
    STATE_DEAD_END,
    STATE_FAULT
};

// ---- Junction type ------------------------------------------
enum JunctionType : uint8_t
{
    JUNCTION_NONE      = 0,
    JUNCTION_LEFT      = 1,   // L-turn left branch only
    JUNCTION_RIGHT     = 2,   // L-turn right branch only
    JUNCTION_T_LEFT    = 3,   // T: front + left
    JUNCTION_T_RIGHT   = 4,   // T: front + right
    JUNCTION_CROSS     = 5,   // full intersection
    JUNCTION_T_BOTH    = 6    // front + both wings (T-top)
};

class TrackDetector
{
public:
    TrackDetector();

    // Call every control loop with the latest sensor data.
    // Returns the updated track state.
    TrackState update(const Sensor& sensor);

    // ---- State accessors ------------------------------------
    TrackState  getState()        const { return _state; }
    JunctionType getJunctionType() const { return _junctionType; }
    const char* stateName()       const;

    // True if the state changed on the last update() call
    bool stateChanged()           const { return _stateChanged; }

    // How long (ms) has the robot been in the current state?
    unsigned long stateAge()      const { return millis() - _stateEnteredAt; }

    // ---- Convenience tests ----------------------------------
    bool isHardTurn()    const;    // TURN_LEFT or TURN_RIGHT
    bool isJunction()    const;    // JUNCTION
    bool isLineLost()    const;    // LINE_LOST, SEARCHING, DEAD_END
    bool isFault()       const;    // FAULT

private:
    TrackState   _state;
    TrackState   _prevState;
    JunctionType _junctionType;
    bool         _stateChanged;
    unsigned long _stateEnteredAt;

    // Temporal persistence counters
    int  _patternPersistCount;
    TrackState _candidateState;
    JunctionType _candidateJunction;

    // Circle detection timer
    unsigned long _wideLineStartMs;
    bool          _wideLine;

    // Dead-end detection
    unsigned long _lineLostAfterLineMs;
    bool          _hadLine;

    // ---- Internal helpers -----------------------------------
    void transitionTo(TrackState newState);
    TrackState classify(const Sensor& sensor);
    JunctionType detectJunction(const Sensor& sensor) const;
    bool isWideLine(const Sensor& sensor) const;
};

// Helper: human-readable track state name
const char* trackStateName(TrackState s);
