#include "TrackLogic.h"

#include "Sensors.h"
#include "Motor.h"
#include "PID.h"


// ============================================================
// GLOBAL TRACK OBJECT
// ============================================================

TrackLogic track;


// ============================================================
// BEGIN
// ============================================================

void TrackLogic::begin()
{
    currentState =
        TRACK_NORMAL;

    stateStartTime =
        millis();

    lineLostTime =
        0;

    lastLinePosition =
        0;

    turnDirection =
        0;

    pid.reset();

    Serial.println(
        "[TRACK] Track logic ready"
    );
}


// ============================================================
// MAIN UPDATE
// ============================================================

void TrackLogic::update()
{
    // --------------------------------------------------------
    // Save the latest known line position
    // --------------------------------------------------------

    if (
        sensors.lineDetected()
    )
    {
        lastLinePosition =
            sensors.position();

        lineLostTime =
            0;
    }
    else
    {
        if (
            lineLostTime == 0
        )
        {
            lineLostTime =
                millis();
        }
    }


    // --------------------------------------------------------
    // Detect special track features
    // --------------------------------------------------------

    detectTrackFeature();


    // --------------------------------------------------------
    // Execute current state
    // --------------------------------------------------------

    switch (
        currentState
    )
    {
        case TRACK_NORMAL:
            handleNormal();
            break;


        case TRACK_CURVE:
            handleCurve();
            break;


        case TRACK_TURN_LEFT:
            handleTurnLeft();
            break;


        case TRACK_TURN_RIGHT:
            handleTurnRight();
            break;


        case TRACK_DASHED:
            handleDashed();
            break;


        case TRACK_JUNCTION:
            handleJunction();
            break;


        case TRACK_CIRCLE:
            handleCircle();
            break;


        case TRACK_SEARCH:
            handleSearch();
            break;


        case TRACK_DEAD_END:
            handleDeadEnd();
            break;
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
    currentState =
        TRACK_NORMAL;

    stateStartTime =
        millis();

    lineLostTime =
        0;

    turnDirection =
        0;

    pid.reset();

    Serial.println(
        "[TRACK] Reset → NORMAL"
    );
}


// ============================================================
// CHANGE STATE
// ============================================================

void TrackLogic::changeState(
    TrackState newState
)
{
    if (
        currentState ==
        newState
    )
    {
        return;
    }


    currentState =
        newState;


    stateStartTime =
        millis();


    switch (
        currentState
    )
    {
        case TRACK_NORMAL:

            Serial.println(
                "[TRACK] NORMAL"
            );

            break;


        case TRACK_CURVE:

            Serial.println(
                "[TRACK] CURVE"
            );

            break;


        case TRACK_TURN_LEFT:

            Serial.println(
                "[TRACK] TURN LEFT"
            );

            break;


        case TRACK_TURN_RIGHT:

            Serial.println(
                "[TRACK] TURN RIGHT"
            );

            break;


        case TRACK_DASHED:

            Serial.println(
                "[TRACK] DASHED GAP"
            );

            break;


        case TRACK_JUNCTION:

            Serial.println(
                "[TRACK] JUNCTION"
            );

            break;


        case TRACK_CIRCLE:

            Serial.println(
                "[TRACK] CIRCLE"
            );

            break;


        case TRACK_SEARCH:

            Serial.println(
                "[TRACK] SEARCH"
            );

            break;


        case TRACK_DEAD_END:

            Serial.println(
                "[TRACK] DEAD END"
            );

            break;
    }
}


// ============================================================
// SENSOR GROUP COUNTS
// ============================================================

int TrackLogic::leftActive()
{
    int count = 0;


    for (
        int i = 0;
        i <= 4;
        i++
    )
    {
        if (
            sensors.strength(i)
            >=
            LINE_STRENGTH_THRESHOLD
        )
        {
            count++;
        }
    }


    return count;
}


// ============================================================
// CENTER GROUP
// ============================================================
//
// C7 and C8 are the actual center pair.
// We include C5-C10 for general forward-line detection.
// ============================================================

int TrackLogic::centerActive()
{
    int count = 0;


    for (
        int i = 5;
        i <= 10;
        i++
    )
    {
        if (
            sensors.strength(i)
            >=
            LINE_STRENGTH_THRESHOLD
        )
        {
            count++;
        }
    }


    return count;
}


// ============================================================
// RIGHT GROUP
// ============================================================

int TrackLogic::rightActive()
{
    int count = 0;


    for (
        int i = 11;
        i < SENSOR_COUNT;
        i++
    )
    {
        if (
            sensors.strength(i)
            >=
            LINE_STRENGTH_THRESHOLD
        )
        {
            count++;
        }
    }


    return count;
}


// ============================================================
// DETECT TRACK FEATURE
// ============================================================
//
// Priority:
//
// 1. Line loss
// 2. Circle / wide pattern
// 3. Junction
// 4. 90° turn
// 5. Curve
// 6. Normal
//
// We deliberately don't react to a single sensor sample.
// ============================================================

void TrackLogic::detectTrackFeature()
{
    // --------------------------------------------------------
    // If already performing a turn, don't overwrite it.
    // --------------------------------------------------------

    if (
        currentState ==
        TRACK_TURN_LEFT
        ||
        currentState ==
        TRACK_TURN_RIGHT
    )
    {
        return;
    }


    // --------------------------------------------------------
    // If searching/recovering, don't overwrite the state.
    // --------------------------------------------------------

    if (
        currentState ==
        TRACK_SEARCH
        ||
        currentState ==
        TRACK_DEAD_END
    )
    {
        return;
    }


    // ========================================================
    // LINE LOSS
    // ========================================================

    if (
        detectLineLoss()
    )
    {
        changeState(
            TRACK_DASHED
        );

        return;
    }


    // ========================================================
    // CIRCLE
    // ========================================================

    if (
        detectCircle()
    )
    {
        changeState(
            TRACK_CIRCLE
        );

        return;
    }


    // ========================================================
    // JUNCTION
    // ========================================================

    if (
        detectJunction()
    )
    {
        changeState(
            TRACK_JUNCTION
        );

        return;
    }


    // ========================================================
    // 90° LEFT
    // ========================================================

    if (
        detectLeftTurn()
    )
    {
        startLeftTurn();

        return;
    }


    // ========================================================
    // 90° RIGHT
    // ========================================================

    if (
        detectRightTurn()
    )
    {
        startRightTurn();

        return;
    }


    // ========================================================
    // CURVE
    // ========================================================

    if (
        abs(
            sensors.position()
        )
        >=
        CURVE_POSITION_LIMIT
    )
    {
        changeState(
            TRACK_CURVE
        );

        return;
    }


    // ========================================================
    // NORMAL
    // ========================================================

    if (
        currentState !=
        TRACK_NORMAL
    )
    {
        changeState(
            TRACK_NORMAL
        );
    }
}


// ============================================================
// LINE LOSS DETECTION
// ============================================================

bool TrackLogic::detectLineLoss()
{
    if (
        sensors.lineDetected()
    )
    {
        return false;
    }


    if (
        lineLostTime == 0
    )
    {
        lineLostTime =
            millis();
    }


    unsigned long lost =
        millis() -
        lineLostTime;


    // --------------------------------------------------------
    // Short loss
    // --------------------------------------------------------

    if (
        lost <
        DASHED_LOSS_MS
    )
    {
        return true;
    }


    // --------------------------------------------------------
    // Long loss
    //
    // Actual search/dead-end handling happens in the state
    // handlers.
    // --------------------------------------------------------

    return true;
}


// ============================================================
// CIRCLE DETECTION
// ============================================================
//
// A circle/cross-circle produces a wide sensor pattern.
//
// Require the pattern to remain present for a short time.
// ============================================================

bool TrackLogic::detectCircle()
{
    static unsigned long wideStart =
        0;


    int active =
        sensors.activeCount();


    if (
        active >=
        CIRCLE_MIN_ACTIVE
    )
    {
        if (
            wideStart == 0
        )
        {
            wideStart =
                millis();
        }


        if (
            millis() -
            wideStart
            >=
            CIRCLE_CONFIRM_MS
        )
        {
            return true;
        }
    }
    else
    {
        wideStart =
            0;
    }


    return false;
}


// ============================================================
// JUNCTION DETECTION
// ============================================================
//
// We consider a junction when:
//
// LEFT branch + center
// OR
// RIGHT branch + center
// OR
// LEFT + RIGHT branches
//
// This avoids declaring a normal curve a junction too easily.
// ============================================================

bool TrackLogic::detectJunction()
{
    int left =
        leftActive();


    int center =
        centerActive();


    int right =
        rightActive();


    bool leftBranch =
        (
            left >=
            JUNCTION_MIN_WING_SENSORS
        );


    bool rightBranch =
        (
            right >=
            JUNCTION_MIN_WING_SENSORS
        );


    bool forward =
        (
            center >= 2
        );


    if (
        (
            leftBranch &&
            forward
        )
        ||
        (
            rightBranch &&
            forward
        )
        ||
        (
            leftBranch &&
            rightBranch
        )
    )
    {
        return true;
    }


    return false;
}


// ============================================================
// LEFT TURN DETECTION
// ============================================================

bool TrackLogic::detectLeftTurn()
{
    int left =
        leftActive();


    int center =
        centerActive();


    int right =
        rightActive();


    if (
        left >=
        TURN_MIN_WING_SENSORS
        &&
        center <=
        TURN_MIN_CENTER_SENSORS
        &&
        right <
        TURN_MIN_WING_SENSORS
    )
    {
        return true;
    }


    return false;
}


// ============================================================
// RIGHT TURN DETECTION
// ============================================================

bool TrackLogic::detectRightTurn()
{
    int left =
        leftActive();


    int center =
        centerActive();


    int right =
        rightActive();


    if (
        right >=
        TURN_MIN_WING_SENSORS
        &&
        center <=
        TURN_MIN_CENTER_SENSORS
        &&
        left <
        TURN_MIN_WING_SENSORS
    )
    {
        return true;
    }


    return false;
}


// ============================================================
// START LEFT TURN
// ============================================================

void TrackLogic::startLeftTurn()
{
    turnDirection =
        -1;


    pid.reset();


    changeState(
        TRACK_TURN_LEFT
    );
}


// ============================================================
// START RIGHT TURN
// ============================================================

void TrackLogic::startRightTurn()
{
    turnDirection =
        +1;


    pid.reset();


    changeState(
        TRACK_TURN_RIGHT
    );
}


// ============================================================
// TURN REACQUIRED
// ============================================================

bool TrackLogic::turnLineReacquired()
{
    int center =
        centerActive();


    unsigned long age =
        millis() -
        stateStartTime;


    if (
        age <
        TURN_MIN_TIME_MS
    )
    {
        return false;
    }


    if (
        center >=
        TURN_REACQUIRE_SENSORS
    )
    {
        return true;
    }


    return false;
}


// ============================================================
// NORMAL LINE FOLLOWING
// ============================================================

void TrackLogic::handleNormal()
{
    if (
        !sensors.lineDetected()
    )
    {
        changeState(
            TRACK_DASHED
        );

        return;
    }


    float error =
        sensors.position();


    float correction =
        pid.calculate(
            error
        );


    int baseSpeed =
        STRAIGHT_SPEED;


    // --------------------------------------------------------
    // Automatically slow down for stronger curves.
    // --------------------------------------------------------

    if (
        abs(error)
        >
        CURVE_POSITION_LIMIT
    )
    {
        baseSpeed =
            CURVE_SPEED;
    }


    int left =
    baseSpeed -
    (int)correction;

    int right =
    baseSpeed +
    (int)correction;


    left =
        constrain(
            left,
            0,
            PWM_MAX
        );


    right =
        constrain(
            right,
            0,
            PWM_MAX
        );


    motor.setMotors(
        left,
        right
    );
}


// ============================================================
// CURVE
// ============================================================

void TrackLogic::handleCurve()
{
    if (
        !sensors.lineDetected()
    )
    {
        changeState(
            TRACK_DASHED
        );

        return;
    }


    float error =
        sensors.position();


    float correction =
        pid.calculate(
            error
        );


    int left =
    CURVE_SPEED -
    (int)correction;

    int right =
    CURVE_SPEED +
    (int)correction;


    left =
        constrain(
            left,
            0,
            PWM_MAX
        );


    right =
        constrain(
            right,
            0,
            PWM_MAX
        );


    motor.setMotors(
        left,
        right
    );


    // --------------------------------------------------------
    // Return to normal when close to center.
    // --------------------------------------------------------

    if (
        abs(error)
        <
        1800
    )
    {
        pid.reset();


        changeState(
            TRACK_NORMAL
        );
    }
}


// ============================================================
// LEFT 90° TURN
// ============================================================

void TrackLogic::handleTurnLeft()
{
    unsigned long age =
        millis() -
        stateStartTime;


    // --------------------------------------------------------
    // Rotate left.
    // --------------------------------------------------------

    motor.setMotors(
        -TURN_SPEED / 2,
        TURN_SPEED
    );


    // --------------------------------------------------------
    // Line found again.
    // --------------------------------------------------------

    if (
        turnLineReacquired()
    )
    {
        pid.reset();


        changeState(
            TRACK_NORMAL
        );


        return;
    }


    // --------------------------------------------------------
    // Safety timeout.
    // --------------------------------------------------------

    if (
        age >
        TURN_TIMEOUT_MS
    )
    {
        changeState(
            TRACK_SEARCH
        );
    }
}


// ============================================================
// RIGHT 90° TURN
// ============================================================

void TrackLogic::handleTurnRight()
{
    unsigned long age =
        millis() -
        stateStartTime;


    // --------------------------------------------------------
    // Rotate right.
    // --------------------------------------------------------

    motor.setMotors(
        TURN_SPEED,
        -TURN_SPEED / 2
    );


    // --------------------------------------------------------
    // Line found again.
    // --------------------------------------------------------

    if (
        turnLineReacquired()
    )
    {
        pid.reset();


        changeState(
            TRACK_NORMAL
        );


        return;
    }


    // --------------------------------------------------------
    // Safety timeout.
    // --------------------------------------------------------

    if (
        age >
        TURN_TIMEOUT_MS
    )
    {
        changeState(
            TRACK_SEARCH
        );
    }
}


// ============================================================
// DASHED LINE / SHORT GAP
// ============================================================

void TrackLogic::handleDashed()
{
    unsigned long lost =
        millis() -
        lineLostTime;


    // --------------------------------------------------------
    // Line came back.
    // --------------------------------------------------------

    if (
        sensors.lineDetected()
    )
    {
        pid.reset();


        changeState(
            TRACK_NORMAL
        );


        return;
    }


    // --------------------------------------------------------
    // Short gap
    // --------------------------------------------------------
    //
    // Continue forward.
    //

    if (
        lost <
        DASHED_LOSS_MS
    )
    {
        motor.setMotors(
            BASE_SPEED,
            BASE_SPEED
        );

        return;
    }


    // --------------------------------------------------------
    // Long loss
    //
    // Start searching.
    // --------------------------------------------------------

    if (
        lost <
        DEAD_END_TIMEOUT_MS
    )
    {
        changeState(
            TRACK_SEARCH
        );

        return;
    }


    // --------------------------------------------------------
    // Very long loss
    //
    // Treat as possible dead end.
    // --------------------------------------------------------

    changeState(
        TRACK_DEAD_END
    );
}


// ============================================================
// JUNCTION
// ============================================================
//
// IMPORTANT:
//
// We do NOT try to guess a complicated route here.
//
// Default behavior:
//
// KEEP MOVING FORWARD.
//
// This makes the controller useful for tracks where the
// intended path continues straight.
//
// L-turns are handled by the turn detector when there is no
// forward line.
// ============================================================

void TrackLogic::handleJunction()
{
    unsigned long age =
        millis() -
        stateStartTime;


    int center =
        centerActive();


    // --------------------------------------------------------
    // Continue straight if forward line exists.
    // --------------------------------------------------------

    if (
        center >= 2
    )
    {
        motor.setMotors(
            JUNCTION_SPEED,
            JUNCTION_SPEED
        );
    }
    else
    {
        // ----------------------------------------------------
        // No forward line.
        //
        // Decide from last known line position.
        // ----------------------------------------------------

        if (
            lastLinePosition < 0
        )
        {
            startLeftTurn();
            return;
        }
        else
        {
            startRightTurn();
            return;
        }
    }


    // --------------------------------------------------------
    // Leave junction once sensor pattern narrows.
    // --------------------------------------------------------

    if (
        age >
        JUNCTION_CONFIRM_MS
        &&
        sensors.activeCount()
        <
        7
    )
    {
        pid.reset();


        changeState(
            TRACK_NORMAL
        );
    }
}


// ============================================================
// CIRCLE / CROSS-CIRCLE
// ============================================================
//
// Slow down and allow the sensor array to remain on the
// circular path.
//
// We deliberately don't force a direction here.
// PID takes over once the pattern becomes narrow again.
// ============================================================

void TrackLogic::handleCircle()
{
    unsigned long age =
        millis() -
        stateStartTime;


    // --------------------------------------------------------
    // Slow forward movement.
    // --------------------------------------------------------

    motor.setMotors(
        CIRCLE_SPEED,
        CIRCLE_SPEED
    );


    // --------------------------------------------------------
    // Pattern has become narrower.
    // --------------------------------------------------------

    if (
        sensors.activeCount()
        <
        CIRCLE_EXIT_ACTIVE
    )
    {
        if (
            sensors.lineDetected()
        )
        {
            pid.reset();


            changeState(
                TRACK_NORMAL
            );


            return;
        }
    }


    // --------------------------------------------------------
    // Safety.
    // --------------------------------------------------------

    if (
        age >
        2500
    )
    {
        if (
            sensors.lineDetected()
        )
        {
            pid.reset();


            changeState(
                TRACK_NORMAL
            );
        }
        else
        {
            changeState(
                TRACK_SEARCH
            );
        }
    }
}


// ============================================================
// SEARCH
// ============================================================
//
// Search toward the last known line position.
// ============================================================

void TrackLogic::handleSearch()
{
    unsigned long age =
        millis() -
        stateStartTime;


    // --------------------------------------------------------
    // Reacquired.
    // --------------------------------------------------------

    if (
        sensors.lineDetected()
    )
    {
        pid.reset();


        changeState(
            TRACK_NORMAL
        );


        return;
    }


    // --------------------------------------------------------
    // Search direction.
    // --------------------------------------------------------

    if (
        lastLinePosition < 0
    )
    {
        motor.setMotors(
            -SEARCH_SPEED,
            SEARCH_SPEED
        );
    }
    else
    {
        motor.setMotors(
            SEARCH_SPEED,
            -SEARCH_SPEED
        );
    }


    // --------------------------------------------------------
    // Search timeout.
    // --------------------------------------------------------

    if (
        age >
        SEARCH_TIMEOUT_MS
    )
    {
        changeState(
            TRACK_DEAD_END
        );
    }
}


// ============================================================
// DEAD END
// ============================================================
//
// Simple recovery:
//
// 1. Reverse
// 2. Rotate
// 3. Search
// ============================================================

void TrackLogic::handleDeadEnd()
{
    unsigned long age =
        millis() -
        stateStartTime;


    // --------------------------------------------------------
    // Reverse
    // --------------------------------------------------------

    if (
        age <
        180
    )
    {
        motor.setMotors(
            -REVERSE_SPEED,
            -REVERSE_SPEED
        );

        return;
    }


    // --------------------------------------------------------
    // Rotate toward last line position
    // --------------------------------------------------------

    if (
        age <
        550
    )
    {
        if (
            lastLinePosition < 0
        )
        {
            motor.setMotors(
                -SEARCH_SPEED,
                SEARCH_SPEED
            );
        }
        else
        {
            motor.setMotors(
                SEARCH_SPEED,
                -SEARCH_SPEED
            );
        }


        return;
    }


    // --------------------------------------------------------
    // Search again
    // --------------------------------------------------------

    changeState(
        TRACK_SEARCH
    );
}


// ============================================================
// DEBUG STATE
// ============================================================

void TrackLogic::printState()
{
    Serial.print(
        "[TRACK] "
    );


    switch (
        currentState
    )
    {
        case TRACK_NORMAL:
            Serial.println(
                "NORMAL"
            );
            break;


        case TRACK_CURVE:
            Serial.println(
                "CURVE"
            );
            break;


        case TRACK_TURN_LEFT:
            Serial.println(
                "TURN LEFT"
            );
            break;


        case TRACK_TURN_RIGHT:
            Serial.println(
                "TURN RIGHT"
            );
            break;


        case TRACK_DASHED:
            Serial.println(
                "DASHED"
            );
            break;


        case TRACK_JUNCTION:
            Serial.println(
                "JUNCTION"
            );
            break;


        case TRACK_CIRCLE:
            Serial.println(
                "CIRCLE"
            );
            break;


        case TRACK_SEARCH:
            Serial.println(
                "SEARCH"
            );
            break;


        case TRACK_DEAD_END:
            Serial.println(
                "DEAD END"
            );
            break;
    }
}