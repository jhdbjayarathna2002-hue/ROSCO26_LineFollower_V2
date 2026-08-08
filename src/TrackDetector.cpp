// ============================================================
// TrackDetector.cpp  —  Track element classification
// ============================================================
// See TrackDetector.h for full documentation.
// ============================================================

#include "TrackDetector.h"

// ============================================================
// Construction
// ============================================================

TrackDetector::TrackDetector()
    : _state(STATE_NORMAL_LINE),
      _prevState(STATE_NORMAL_LINE),
      _junctionType(JUNCTION_NONE),
      _stateChanged(false),
      _stateEnteredAt(0),
      _patternPersistCount(0),
      _candidateState(STATE_NORMAL_LINE),
      _candidateJunction(JUNCTION_NONE),
      _wideLineStartMs(0),
      _wideLine(false),
      _lineLostAfterLineMs(0),
      _hadLine(false)
{
}

// ============================================================
// trackStateName()  —  free function for human-readable names
// ============================================================

const char* trackStateName(TrackState s)
{
    switch (s)
    {
        case STATE_NORMAL_LINE:  return "NORMAL_LINE";
        case STATE_CURVE:        return "CURVE";
        case STATE_TURN_LEFT:    return "TURN_LEFT";
        case STATE_TURN_RIGHT:   return "TURN_RIGHT";
        case STATE_JUNCTION:     return "JUNCTION";
        case STATE_CIRCLE:       return "CIRCLE";
        case STATE_DASHED_LINE:  return "DASHED_LINE";
        case STATE_LINE_LOST:    return "LINE_LOST";
        case STATE_SEARCHING:    return "SEARCHING";
        case STATE_RECOVERY:     return "RECOVERY";
        case STATE_DEAD_END:     return "DEAD_END";
        case STATE_FAULT:        return "FAULT";
        default:                 return "UNKNOWN";
    }
}

const char* TrackDetector::stateName() const
{
    return trackStateName(_state);
}

// ============================================================
// transitionTo()
// ============================================================

void TrackDetector::transitionTo(TrackState newState)
{
    if (newState == _state) return;

    _prevState      = _state;
    _state          = newState;
    _stateChanged   = true;
    _stateEnteredAt = millis();

#ifdef DEBUG_TRACK
    Serial.printf("[TRACK] %s -> %s\n",
                  trackStateName(_prevState),
                  trackStateName(_state));
#endif
}

// ============================================================
// isWideLine()
// Returns true when a very large number of sensors are active,
// suggesting a circle/wide section rather than a simple line.
// ============================================================

bool TrackDetector::isWideLine(const Sensor& sensor) const
{
    return sensor.activeSensorCount >= CIRCLE_MIN_SENSORS;
}

// ============================================================
// detectJunction()
// Inspect left wing, centre, and right wing to determine the
// type of branching structure.
// ============================================================

JunctionType TrackDetector::detectJunction(const Sensor& sensor) const
{
    int leftCount   = sensor.countActive(LEFT_GROUP_START,   LEFT_GROUP_END);
    int centerCount = sensor.countActive(CENTER_GROUP_START, CENTER_GROUP_END);
    int rightCount  = sensor.countActive(RIGHT_GROUP_START,  RIGHT_GROUP_END);

    bool hasLeft   = (leftCount   >= JUNCTION_WING_MIN);
    bool hasFront  = (centerCount >= 2);
    bool hasRight  = (rightCount  >= JUNCTION_WING_MIN);

    if (hasLeft && hasRight && hasFront) return JUNCTION_CROSS;
    if (hasLeft && hasRight)             return JUNCTION_T_BOTH;
    if (hasLeft  && hasFront)            return JUNCTION_T_LEFT;
    if (hasRight && hasFront)            return JUNCTION_T_RIGHT;
    if (hasLeft  && !hasFront)           return JUNCTION_LEFT;
    if (hasRight && !hasFront)           return JUNCTION_RIGHT;

    return JUNCTION_NONE;
}

// ============================================================
// classify()
// Compute the candidate state from the current sensor snapshot.
// Does NOT apply temporal filtering here — that is done in update().
// ============================================================

TrackState TrackDetector::classify(const Sensor& sensor)
{
    const bool linePresent = sensor.lineDetected();
    const int  active      = sensor.activeSensorCount;
    const int  pos         = sensor.lastPosition;

    // ---- No line at all -----------------------------------------------
    if (!linePresent)
        return STATE_LINE_LOST;

    // ---- Very wide line — potential circle / cross-circle -------------
    if (isWideLine(sensor))
        return STATE_CIRCLE;

    // ---- Junction / intersection check --------------------------------
    _candidateJunction = detectJunction(sensor);

    if (_candidateJunction != JUNCTION_NONE)
        return STATE_JUNCTION;

    // ---- Hard turn: strong one-sided activation, no centre -----------
    int leftCount   = sensor.countActive(LEFT_GROUP_START,  LEFT_GROUP_END);
    int centerCount = sensor.countActive(CENTER_GROUP_START, CENTER_GROUP_END);
    int rightCount  = sensor.countActive(RIGHT_GROUP_START, RIGHT_GROUP_END);

    const int leftGroupSize   = LEFT_GROUP_END  - LEFT_GROUP_START  + 1;  // 5
    const int rightGroupSize  = RIGHT_GROUP_END - RIGHT_GROUP_START + 1;  // 5

    bool strongLeft  = (leftCount  >= (int)(leftGroupSize  * TURN_ACTIVATION_RATIO));
    bool strongRight = (rightCount >= (int)(rightGroupSize * TURN_ACTIVATION_RATIO));
    bool noCenter    = (centerCount <= 1);

    if (strongLeft  && noCenter && !strongRight) return STATE_TURN_LEFT;
    if (strongRight && noCenter && !strongLeft)  return STATE_TURN_RIGHT;

    // ---- Curve: position significantly off-centre -------------------
    if (pos < -2000 || pos > +2000)
        return STATE_CURVE;

    // ---- Normal line ------------------------------------------------
    return STATE_NORMAL_LINE;
}

// ============================================================
// update()  —  main entry point, call every control loop
// ============================================================

TrackState TrackDetector::update(const Sensor& sensor)
{
    _stateChanged = false;

    bool linePresent = sensor.lineDetected();

    // ---- Track whether we "had" a line (for dead-end detection) ---
    if (linePresent)
    {
        _hadLine             = true;
        _lineLostAfterLineMs = 0;
    }

    // ---- Wide-line circle timer -----------------------------------
    if (isWideLine(sensor))
    {
        if (!_wideLine)
        {
            _wideLineStartMs = millis();
            _wideLine        = true;
        }
    }
    else
    {
        _wideLine = false;
        _wideLineStartMs = 0;
    }

    // ---- Don't interrupt FAULT state ------------------------------
    if (_state == STATE_FAULT)
        return _state;

    // ---- Classify the candidate state ----------------------------
    TrackState candidate = classify(sensor);

    // ---- Circle requires persistence in time ---------------------
    if (candidate == STATE_CIRCLE)
    {
        if (!_wideLine ||
            (millis() - _wideLineStartMs < (unsigned long)CIRCLE_PERSIST_MS))
        {
            // Not yet confirmed — fall back to current state or NORMAL
            candidate = (_state == STATE_CIRCLE) ? STATE_CIRCLE : STATE_NORMAL_LINE;
        }
    }

    // ---- Dead-end detection: line was present, now fully absent ---
    if (!linePresent && _hadLine)
    {
        if (_lineLostAfterLineMs == 0)
            _lineLostAfterLineMs = millis();

        if ((millis() - _lineLostAfterLineMs) > (unsigned long)DEAD_END_PERSIST_MS
            && _state == STATE_LINE_LOST)
        {
            transitionTo(STATE_DEAD_END);
            return _state;
        }
    }
    else if (linePresent)
    {
        _lineLostAfterLineMs = 0;
    }

    // ---- Temporal persistence filter -----------------------------
    // Require PATTERN_PERSIST_COUNT consecutive identical candidates
    // before committing to a new state (avoids single-scan glitches).
    if (candidate == _candidateState)
    {
        _patternPersistCount++;
    }
    else
    {
        _candidateState      = candidate;
        _patternPersistCount = 1;
    }

    // Only transition when the candidate is stable enough
    // OR if it is safety-critical (LINE_LOST / FAULT)
    if (_patternPersistCount >= PATTERN_PERSIST_COUNT
        || candidate == STATE_LINE_LOST
        || candidate == STATE_FAULT)
    {
        // Update junction type if entering JUNCTION
        if (candidate == STATE_JUNCTION)
            _junctionType = _candidateJunction;

        transitionTo(candidate);
    }

    return _state;
}

// ============================================================
// Convenience predicates
// ============================================================

bool TrackDetector::isHardTurn() const
{
    return _state == STATE_TURN_LEFT || _state == STATE_TURN_RIGHT;
}

bool TrackDetector::isJunction() const
{
    return _state == STATE_JUNCTION;
}

bool TrackDetector::isLineLost() const
{
    return _state == STATE_LINE_LOST
        || _state == STATE_SEARCHING
        || _state == STATE_DEAD_END;
}

bool TrackDetector::isFault() const
{
    return _state == STATE_FAULT;
}
