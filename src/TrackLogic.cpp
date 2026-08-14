#include "TrackLogic.h"

#include "Sensors.h"
#include "Motor.h"
#include "PID.h"


// ============================================================
// GLOBAL OBJECT
// ============================================================

TrackLogic track;


// ============================================================
// BEGIN
// ============================================================

void TrackLogic::begin()
{
    currentState            = TRACK_NORMAL;
    stateStartTime          = millis();
    lineLostTime            = 0;
    lastLinePosition        = 0;
    turnCandidateTime       = 0;
    turnCandidateDir        = 0;
    curveCandidateTime      = 0;
    curveCandidateActive    = false;
    junctionCandidateTime   = 0;
    junctionCandidateActive = false;
    circleCandidateTime     = 0;
    circleCandidateActive   = false;
    lastDebugTime           = 0;

    pid.reset();

    Serial.println("[TRACK] Track logic ready");
}


// ============================================================
// UPDATE  —  called every loop iteration after sensors.update()
// ============================================================

void TrackLogic::update()
{
    switch (currentState)
    {
        case TRACK_NORMAL:          handleNormal();          break;
        case TRACK_CURVE:           handleCurve();           break;
        case TRACK_TURN_LEFT:       handleTurnLeft();        break;
        case TRACK_TURN_RIGHT:      handleTurnRight();       break;
        case TRACK_GAP:             handleGap();             break;
        case TRACK_SEARCH:          handleSearch();          break;
        case TRACK_DEAD_END:        handleDeadEnd();         break;
        case TRACK_JUNCTION:        handleJunction();        break;
        case TRACK_CIRCLE:          handleCircle();          break;
        case TRACK_TASK01_COMPLETE: handleTask01Complete();  break;
    }
}


// ============================================================
// STATE
// ============================================================

TrackState TrackLogic::state()
{
    return currentState;
}


// ============================================================
// RESET
// ============================================================

void TrackLogic::reset()
{
    currentState            = TRACK_NORMAL;
    stateStartTime          = millis();
    lineLostTime            = 0;
    lastLinePosition        = 0;
    turnCandidateDir        = 0;
    turnCandidateTime       = 0;
    curveCandidateActive    = false;
    junctionCandidateActive = false;
    circleCandidateActive   = false;

    pid.reset();
}


// ============================================================
// CHANGE STATE
// ============================================================

void TrackLogic::changeState(TrackState newState)
{
    if (currentState == newState) return;

    // Reset all candidate timers on any state change
    turnCandidateDir        = 0;
    curveCandidateActive    = false;
    junctionCandidateActive = false;
    circleCandidateActive   = false;

    currentState   = newState;
    stateStartTime = millis();

    printState();
}


// ============================================================
// SENSOR ZONES
// ============================================================
//
// LEFT   = C0..C4   (5 sensors)
// CENTER = C5..C10  (6 sensors, contains C7+C8 center pair)
// RIGHT  = C11..C15 (5 sensors)
//
// ============================================================

int TrackLogic::leftActive()
{
    int count = 0;

    for (int i = 0; i <= 4; i++)
    {
        if (sensors.strength(i) >= LINE_STRENGTH_THRESHOLD)
        {
            count++;
        }
    }

    return count;
}


int TrackLogic::centerActive()
{
    int count = 0;

    for (int i = 5; i <= 10; i++)
    {
        if (sensors.strength(i) >= LINE_STRENGTH_THRESHOLD)
        {
            count++;
        }
    }

    return count;
}


int TrackLogic::rightActive()
{
    int count = 0;

    for (int i = 11; i <= 15; i++)
    {
        if (sensors.strength(i) >= LINE_STRENGTH_THRESHOLD)
        {
            count++;
        }
    }

    return count;
}


// ============================================================
// FEATURE DETECTION
// ============================================================

bool TrackLogic::detectLineLoss()
{
    return !sensors.lineDetected();
}


// Circle: >= 13 sensors simultaneously active.
// Only possible at a circular section entry (400-450 mm wide loop).

bool TrackLogic::detectCircle()
{
    return sensors.activeCount() >= CIRCLE_MIN_ACTIVE;
}


// Circle exit: active count drops below threshold.

bool TrackLogic::detectCircleExit()
{
    return sensors.activeCount() < CIRCLE_EXIT_ACTIVE;
}


// Junction: center line + at least one wing strongly active.
// This distinguishes a T/L/cross from a pure 90-degree turn
// (which has wing sensors active but no center).

bool TrackLogic::detectJunction()
{
    int L     = leftActive();
    int C     = centerActive();
    int R     = rightActive();
    int total = sensors.activeCount();

    return (
        C     >= JUNCTION_MIN_CENTER_SENSORS &&
        total >= JUNCTION_TOTAL_MIN &&
        (L >= JUNCTION_MIN_WING_SENSORS || R >= JUNCTION_MIN_WING_SENSORS)
    );
}


// Left turn: line has moved entirely onto the LEFT wing.
// CENTER is almost clear, RIGHT is empty.

bool TrackLogic::detectLeftTurn()
{
    return (
        leftActive()   >= TURN_MIN_WING_SENSORS &&
        centerActive() <= TURN_MIN_CENTER_SENSORS &&
        rightActive()  == 0
    );
}


bool TrackLogic::detectRightTurn()
{
    return (
        rightActive()  >= TURN_MIN_WING_SENSORS &&
        centerActive() <= TURN_MIN_CENTER_SENSORS &&
        leftActive()   == 0
    );
}


// Curve: position magnitude exceeds CURVE_POSITION_LIMIT.
// With sensor weights -7500..+7500, the threshold of 6000 only
// fires for genuinely sharp curves.

bool TrackLogic::detectCurve()
{
    int pos = sensors.position();

    return (
        pos >  CURVE_POSITION_LIMIT ||
        pos < -CURVE_POSITION_LIMIT
    );
}


// ============================================================
// TURN REACQUIRE
// ============================================================
//
// After pivoting, we are done when C6..C9 (surrounding the
// C7+C8 center pair) see the line again.

bool TrackLogic::turnReacquired()
{
    int seen = 0;

    for (int i = 6; i <= 9; i++)
    {
        if (sensors.strength(i) >= LINE_STRENGTH_THRESHOLD)
        {
            seen++;
        }
    }

    return (seen >= TURN_REACQUIRE_SENSORS);
}


// ============================================================
// TURN HELPERS
// ============================================================

void TrackLogic::startLeftTurn()
{
    pid.reset();
    changeState(TRACK_TURN_LEFT);
}


void TrackLogic::startRightTurn()
{
    pid.reset();
    changeState(TRACK_TURN_RIGHT);
}


// ============================================================
// HANDLE NORMAL
// ============================================================
//
// Detection priority (checked every control cycle):
//
//   1. Line lost                -> GAP
//   2. Circle (13+ sensors)     -> CIRCLE   (CIRCLE_CONFIRM_MS persistence)
//   3. Junction (center + wing) -> JUNCTION (JUNCTION_CONFIRM_MS persistence)
//   4. 90-degree turn           -> TURN_L/R (TURN_MIN_TIME_MS persistence)
//   5. Curve (pos > 6000)       -> CURVE    (CURVE_CONFIRM_MS persistence)
//   6. Normal PID following
//
// Every feature requires its pattern to persist across multiple
// control cycles before the state is committed. One noisy sensor
// reading cannot cause a state transition.
// ============================================================

void TrackLogic::handleNormal()
{
    // --------------------------------------------------------
    // 1. Line lost -> GAP
    // --------------------------------------------------------

    if (detectLineLoss())
    {
        lineLostTime            = millis();
        curveCandidateActive    = false;
        junctionCandidateActive = false;
        circleCandidateActive   = false;
        turnCandidateDir        = 0;

        changeState(TRACK_GAP);

        motor.setMotors(STRAIGHT_SPEED, STRAIGHT_SPEED);

        return;
    }


    // Save last known good position for search/dead-end recovery
    lastLinePosition = sensors.position();


    // --------------------------------------------------------
    // 2. Circle detection (highest priority after line loss)
    // --------------------------------------------------------

    if (detectCircle())
    {
        if (!circleCandidateActive)
        {
            circleCandidateActive = true;
            circleCandidateTime   = millis();
        }
        else if (millis() - circleCandidateTime >= (unsigned long)CIRCLE_CONFIRM_MS)
        {
            circleCandidateActive = false;
            pid.reset();
            changeState(TRACK_CIRCLE);
            return;
        }
    }
    else
    {
        circleCandidateActive = false;
    }


    // --------------------------------------------------------
    // 3. Junction detection (center + wing simultaneously active)
    // --------------------------------------------------------

    if (detectJunction())
    {
        if (!junctionCandidateActive)
        {
            junctionCandidateActive = true;
            junctionCandidateTime   = millis();
        }
        else if (millis() - junctionCandidateTime >= (unsigned long)JUNCTION_CONFIRM_MS)
        {
            junctionCandidateActive = false;
            changeState(TRACK_JUNCTION);
            return;
        }
    }
    else
    {
        junctionCandidateActive = false;
    }


    // --------------------------------------------------------
    // 4. Turn detection (wing only, no center)
    // --------------------------------------------------------

    bool leftTurn  = detectLeftTurn();
    bool rightTurn = detectRightTurn();

    if (leftTurn || rightTurn)
    {
        int dir = leftTurn ? -1 : +1;

        if (turnCandidateDir != dir)
        {
            // New turn direction candidate — start timing
            turnCandidateDir  = dir;
            turnCandidateTime = millis();
        }
        else if (millis() - turnCandidateTime >= (unsigned long)TURN_MIN_TIME_MS)
        {
            // Pattern held for TURN_MIN_TIME_MS — commit to turn
            turnCandidateDir = 0;

            if (dir == -1) startLeftTurn();
            else           startRightTurn();

            return;
        }
    }
    else
    {
        // Pattern dropped before timer expired — reset candidate
        turnCandidateDir = 0;
    }


    // --------------------------------------------------------
    // 5. Curve detection (position magnitude > CURVE_POSITION_LIMIT)
    // --------------------------------------------------------

    if (detectCurve())
    {
        if (!curveCandidateActive)
        {
            curveCandidateActive = true;
            curveCandidateTime   = millis();
        }
        else if (millis() - curveCandidateTime >= (unsigned long)CURVE_CONFIRM_MS)
        {
            curveCandidateActive = false;
            changeState(TRACK_CURVE);
            return;
        }
    }
    else
    {
        curveCandidateActive = false;
    }


    // --------------------------------------------------------
    // 6. Normal PID following at STRAIGHT_SPEED
    // --------------------------------------------------------

    float error      = (float)(sensors.position() * POSITION_SIGN);
    float correction = pid.calculate(error);

    int left  = STRAIGHT_SPEED - (int)correction;
    int right = STRAIGHT_SPEED + (int)correction;

    left  = constrain(left,  -PWM_MAX, PWM_MAX);
    right = constrain(right, -PWM_MAX, PWM_MAX);

    motor.setMotors(left, right);


#ifdef DEBUG_TRACK

    unsigned long now = millis();

    if (now - lastDebugTime >= (unsigned long)DEBUG_PRINT_RATE_MS)
    {
        lastDebugTime = now;

        Serial.print("[NORMAL] pos=");
        Serial.print(sensors.position());
        Serial.print(" act=");
        Serial.print(sensors.activeCount());
        Serial.print(" L=");
        Serial.print(left);
        Serial.print(" R=");
        Serial.println(right);
    }

#endif
}


// ============================================================
// HANDLE CURVE
// ============================================================
//
// PID following at CURVE_SPEED (slower).
//
// Priorities inside curve:
//   1. Line lost   -> GAP
//   2. Circle      -> CIRCLE
//   3. Turn        -> TURN_L/R
//   4. Pos drops   -> NORMAL
//   5. PID at CURVE_SPEED

void TrackLogic::handleCurve()
{
    // --------------------------------------------------------
    // Line lost -> GAP
    // --------------------------------------------------------

    if (detectLineLoss())
    {
        lineLostTime          = millis();
        circleCandidateActive = false;

        changeState(TRACK_GAP);

        motor.setMotors(STRAIGHT_SPEED, STRAIGHT_SPEED);

        return;
    }


    lastLinePosition = sensors.position();


    // --------------------------------------------------------
    // Circle check while curving
    // --------------------------------------------------------

    if (detectCircle())
    {
        if (!circleCandidateActive)
        {
            circleCandidateActive = true;
            circleCandidateTime   = millis();
        }
        else if (millis() - circleCandidateTime >= (unsigned long)CIRCLE_CONFIRM_MS)
        {
            circleCandidateActive = false;
            pid.reset();
            changeState(TRACK_CIRCLE);
            return;
        }
    }
    else
    {
        circleCandidateActive = false;
    }


    // --------------------------------------------------------
    // Turn check while curving (same persistence guard)
    // --------------------------------------------------------

    bool leftTurn  = detectLeftTurn();
    bool rightTurn = detectRightTurn();

    if (leftTurn || rightTurn)
    {
        int dir = leftTurn ? -1 : +1;

        if (turnCandidateDir != dir)
        {
            turnCandidateDir  = dir;
            turnCandidateTime = millis();
        }
        else if (millis() - turnCandidateTime >= (unsigned long)TURN_MIN_TIME_MS)
        {
            turnCandidateDir = 0;

            if (dir == -1) startLeftTurn();
            else           startRightTurn();

            return;
        }
    }
    else
    {
        turnCandidateDir = 0;
    }


    // --------------------------------------------------------
    // Return to NORMAL if position is no longer extreme
    // --------------------------------------------------------

    int pos = sensors.position();

    if (pos >= -CURVE_POSITION_LIMIT && pos <= CURVE_POSITION_LIMIT)
    {
        changeState(TRACK_NORMAL);
        return;
    }


    // --------------------------------------------------------
    // PID at curve speed
    // --------------------------------------------------------

    float error      = (float)(pos * POSITION_SIGN);
    float correction = pid.calculate(error);

    int left  = CURVE_SPEED - (int)correction;
    int right = CURVE_SPEED + (int)correction;

    left  = constrain(left,  -PWM_MAX, PWM_MAX);
    right = constrain(right, -PWM_MAX, PWM_MAX);

    motor.setMotors(left, right);
}


// ============================================================
// HANDLE GAP  (dashed line / brief line loss)
// ============================================================
//
//   < DASHED_LOSS_MS  : coast straight (dashed gap tolerance)
//   >= DASHED_LOSS_MS : genuinely lost -> SEARCH
//
// If line reappears at any point -> NORMAL immediately.

void TrackLogic::handleGap()
{
    if (sensors.lineDetected())
    {
        pid.reset();
        changeState(TRACK_NORMAL);
        return;
    }

    unsigned long lost = millis() - lineLostTime;

    if (lost < (unsigned long)DASHED_LOSS_MS)
    {
        motor.setMotors(STRAIGHT_SPEED, STRAIGHT_SPEED);
        return;
    }

    changeState(TRACK_DEAD_END);
}


// ============================================================
// HANDLE SEARCH
// ============================================================
//
// Spin toward last known line position.
// SEARCH_TIMEOUT_MS exceeded -> DEAD_END.

void TrackLogic::handleSearch()
{
    if (sensors.lineDetected())
    {
        pid.reset();
        changeState(TRACK_NORMAL);
        return;
    }

    unsigned long elapsed = millis() - stateStartTime;

    if (elapsed >= (unsigned long)SEARCH_TIMEOUT_MS)
    {
        motor.stop();
        changeState(TRACK_DEAD_END);
        return;
    }

    // Spin toward side where line was last seen
    if (lastLinePosition >= 0)
        motor.setMotors( SEARCH_SPEED, -SEARCH_SPEED);
    else
        motor.setMotors(-SEARCH_SPEED,  SEARCH_SPEED);
}


// ============================================================
// HANDLE DEAD END
// ============================================================
//
// Phase 1 (< DEAD_END_TIMEOUT_MS): reverse.
// Phase 2 (>= DEAD_END_TIMEOUT_MS): slow spin opposite to
//   last known direction, looking for line.

void TrackLogic::handleDeadEnd()
{
    // If we've found the line during our spin, resume following
    if (sensors.lineDetected())
    {
        pid.reset();
        changeState(TRACK_NORMAL);
        return;
    }

    // Spin in place (180 turn) to find the line behind us.
    // We use TURN_SPEED which is slightly faster than SEARCH_SPEED.
    if (lastLinePosition >= 0)
    {
        motor.setMotors(-TURN_SPEED,  TURN_SPEED);
    }
    else
    {
        motor.setMotors( TURN_SPEED, -TURN_SPEED);
    }
}


// ============================================================
// HANDLE TURN LEFT
// ============================================================
//
// 1. Pivot left (L=−TURN_SPEED, R=+TURN_SPEED) until either:
//    a. turnReacquired() after TURN_MIN_TIME_MS -> NORMAL
//    b. TURN_TIMEOUT_MS exceeded             -> SEARCH (fallback)

void TrackLogic::handleTurnLeft()
{
    unsigned long elapsed = millis() - stateStartTime;

    // Safety timeout
    if (elapsed >= (unsigned long)TURN_TIMEOUT_MS)
    {
        motor.stop();
        changeState(TRACK_SEARCH);
        return;
    }

    // Reacquire check (only after minimum pivot time)
    if (elapsed >= (unsigned long)TURN_MIN_TIME_MS && turnReacquired())
    {
        motor.stop();
        pid.reset();
        changeState(TRACK_NORMAL);
        return;
    }

    // Pivot left
    motor.setMotors(-TURN_SPEED, TURN_SPEED);
}


// ============================================================
// HANDLE TURN RIGHT
// ============================================================

void TrackLogic::handleTurnRight()
{
    unsigned long elapsed = millis() - stateStartTime;

    // Safety timeout
    if (elapsed >= (unsigned long)TURN_TIMEOUT_MS)
    {
        motor.stop();
        changeState(TRACK_SEARCH);
        return;
    }

    // Reacquire check
    if (elapsed >= (unsigned long)TURN_MIN_TIME_MS && turnReacquired())
    {
        motor.stop();
        pid.reset();
        changeState(TRACK_NORMAL);
        return;
    }

    // Pivot right
    motor.setMotors(TURN_SPEED, -TURN_SPEED);
}


// ============================================================
// HANDLE JUNCTION
// ============================================================
//
// Navigation strategy is set by JUNCTION_NAV_DEFAULT in Config.h:
//
//   JUNCTION_NAV_STRAIGHT (0):
//     Continue PID through the junction at JUNCTION_SPEED.
//     Exit when the wide sensor pattern clears.
//
//   JUNCTION_NAV_LEFT (1):
//     Creep forward into junction then execute a left pivot.
//
//   JUNCTION_NAV_RIGHT (2):
//     Creep forward into junction then execute a right pivot.
//
// Change JUNCTION_NAV_DEFAULT in Config.h without touching
// any other file.

void TrackLogic::handleJunction()
{
    // If line is genuinely lost, fall back to GAP
    if (detectLineLoss())
    {
        lineLostTime = millis();
        changeState(TRACK_GAP);
        motor.setMotors(STRAIGHT_SPEED, STRAIGHT_SPEED);
        return;
    }

    lastLinePosition = sensors.position();

    unsigned long elapsed = millis() - stateStartTime;


    // ----------------------------------------------------------
    // STRAIGHT THROUGH (default)
    // ----------------------------------------------------------

    if (JUNCTION_NAV_DEFAULT == JUNCTION_NAV_STRAIGHT)
    {
        // Exit when the wide pattern has cleared (junction is behind us)
        if (elapsed > (unsigned long)JUNCTION_CONFIRM_MS &&
            sensors.activeCount() < (JUNCTION_TOTAL_MIN - 2))
        {
            pid.reset();
            changeState(TRACK_NORMAL);
            return;
        }

        float error      = (float)(sensors.position() * POSITION_SIGN);
        float correction = pid.calculate(error);

        int left  = JUNCTION_SPEED - (int)correction;
        int right = JUNCTION_SPEED + (int)correction;

        left  = constrain(left,  -PWM_MAX, PWM_MAX);
        right = constrain(right, -PWM_MAX, PWM_MAX);

        motor.setMotors(left, right);

        return;
    }


    // ----------------------------------------------------------
    // TURN LEFT
    // ----------------------------------------------------------

    if (JUNCTION_NAV_DEFAULT == JUNCTION_NAV_LEFT)
    {
        // Creep slowly into the junction for 250 ms, then pivot
        if (elapsed < 250)
        {
            motor.setMotors(JUNCTION_SPEED, JUNCTION_SPEED);
            return;
        }

        startLeftTurn();
        return;
    }


    // ----------------------------------------------------------
    // TURN RIGHT
    // ----------------------------------------------------------

    if (JUNCTION_NAV_DEFAULT == JUNCTION_NAV_RIGHT)
    {
        if (elapsed < 250)
        {
            motor.setMotors(JUNCTION_SPEED, JUNCTION_SPEED);
            return;
        }

        startRightTurn();
        return;
    }
}


// ============================================================
// HANDLE CIRCLE
// ============================================================
//
// The robot is traversing a circular section (~400-450 mm).
// Follow using PID at CIRCLE_SPEED.
//
// Exit condition: activeCount < CIRCLE_EXIT_ACTIVE must hold
// for CIRCLE_FOLLOW_EXIT_MS to avoid premature exit on
// momentary sensor variation.

void TrackLogic::handleCircle()
{
    // If line is genuinely lost mid-circle, go to GAP
    if (detectLineLoss())
    {
        lineLostTime          = millis();
        circleCandidateActive = false;

        changeState(TRACK_GAP);
        motor.setMotors(STRAIGHT_SPEED, STRAIGHT_SPEED);
        return;
    }

    lastLinePosition = sensors.position();


    // --------------------------------------------------------
    // Circle exit detection (confirmed over CIRCLE_FOLLOW_EXIT_MS)
    // --------------------------------------------------------

    if (detectCircleExit())
    {
        if (!circleCandidateActive)
        {
            // Start exit confirmation timer
            circleCandidateActive = true;
            circleCandidateTime   = millis();
        }
        else if (millis() - circleCandidateTime >= (unsigned long)CIRCLE_FOLLOW_EXIT_MS)
        {
            // Exit confirmed
            circleCandidateActive = false;
            pid.reset();
            changeState(TRACK_NORMAL);
            return;
        }
    }
    else
    {
        // Still in circle — reset exit candidate
        circleCandidateActive = false;
    }


    // --------------------------------------------------------
    // PID at circle speed
    // --------------------------------------------------------

    float error      = (float)sensors.position();
    float correction = pid.calculate(error);

    int left  = CIRCLE_SPEED - (int)correction;
    int right = CIRCLE_SPEED + (int)correction;

    left  = constrain(left,  -PWM_MAX, PWM_MAX);
    right = constrain(right, -PWM_MAX, PWM_MAX);

    motor.setMotors(left, right);
}


// ============================================================
// HANDLE TASK 01 COMPLETE
// ============================================================
//
// Terminal state. Robot has reached Check Point 1.
// Stop and disable motors. Transition to next task subsystem
// when the overall robot controller is ready.

void TrackLogic::handleTask01Complete()
{
    motor.stop();
    motor.disable();
}


// ============================================================
// PRINT STATE  —  called by changeState() on every transition
// ============================================================

void TrackLogic::printState()
{
    Serial.print("[TRACK] ");

    switch (currentState)
    {
        case TRACK_NORMAL:
            Serial.println("NORMAL");
            break;

        case TRACK_CURVE:
            Serial.println("CURVE");
            break;

        case TRACK_TURN_LEFT:
            Serial.println("TURN LEFT");
            break;

        case TRACK_TURN_RIGHT:
            Serial.println("TURN RIGHT");
            break;

        case TRACK_GAP:
            Serial.println("GAP");
            break;

        case TRACK_SEARCH:
            Serial.println("SEARCH");
            break;

        case TRACK_DEAD_END:
            Serial.println("DEAD END");
            break;

        case TRACK_JUNCTION:
            Serial.println("JUNCTION");
            break;

        case TRACK_CIRCLE:
            Serial.println("CIRCLE");
            break;

        case TRACK_TASK01_COMPLETE:
            Serial.println("TASK 01 COMPLETE");
            break;
    }
}

