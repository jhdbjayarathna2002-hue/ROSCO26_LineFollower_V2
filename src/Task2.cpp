#include "Task2.h"

#include "Config.h"
#include "Motor.h"
#include "PID.h"
#include "Sensors.h"
#include "ToFSensors.h"
#include "TrackLogic.h"

Task2Controller task2;

void Task2Controller::begin()
{
    currentState = TASK2_IDLE;
    stateAfterSettle = TASK2_IDLE;
    stateStartTime = millis();
    junctionCandidateTime = 0;
    lastDebugTime = 0;
    previousWallError = 0.0f;
    invalidFrontCount = 0;

    Serial.println("[TASK2] Controller ready");
}

bool Task2Controller::start()
{
    if (!tofSensors.isReady())
    {
        setError("ToF sensors are not ready");
        return false;
    }

    motor.stop();
    motor.enable();

    previousWallError = 0.0f;
    invalidFrontCount = 0;
    junctionCandidateTime = 0;
    pid.reset();

    // Test starting position:
    // Robot is on the line after Task 1 and facing Junction 1.
    changeState(TASK2_FOLLOW_TO_JUNCTION_1);
    return true;
}

void Task2Controller::stop()
{
    motor.stop();
    currentState = TASK2_IDLE;
    Serial.println("[TASK2] Stopped");
}

Task2State Task2Controller::state() const
{
    return currentState;
}

void Task2Controller::changeState(Task2State newState)
{
    currentState = newState;
    stateStartTime = millis();
    previousWallError = 0.0f;
    invalidFrontCount = 0;
    junctionCandidateTime = 0;

    printState();
}

void Task2Controller::beginSettle(Task2State nextState)
{
    motor.stop();
    stateAfterSettle = nextState;
    changeState(TASK2_SETTLE);
}

void Task2Controller::updateSettle()
{
    motor.stop();

    if (millis() - stateStartTime >= (unsigned long)TASK2_TURN_SETTLE_MS)
    {
        changeState(stateAfterSettle);

        if (currentState == TASK2_FOLLOW_TO_JUNCTION_1 ||
            currentState == TASK2_FOLLOW_TO_JUNCTION_2)
        {
            pid.reset();
        }

        if (currentState == TASK2_FOLLOW_TO_RAMP)
        {
            track.reset();
        }
    }
}

void Task2Controller::updateTimedTurn(
    bool clockwise,
    unsigned long turnTimeMS,
    Task2State nextState
)
{
    if (millis() - stateStartTime >= turnTimeMS)
    {
        beginSettle(nextState);
        return;
    }

    if (clockwise)
    {
        // Clockwise/right pivot.
        motor.setMotors(TASK2_TURN_SPEED, -TASK2_TURN_SPEED);
    }
    else
    {
        // Counter-clockwise/left pivot.
        motor.setMotors(-TASK2_TURN_SPEED, TASK2_TURN_SPEED);
    }
}

void Task2Controller::updateRoomForward(Task2State reverseState)
{
    if (!tofSensors.update())
    {
        return;
    }

    int left = tofSensors.leftMM();
    int front = tofSensors.frontMM();
    int right = tofSensors.rightMM();

    if (!tofSensors.frontValid())
    {
        invalidFrontCount++;

        if (invalidFrontCount >= 3)
        {
            setError("Front ToF invalid for three readings");
        }

        return;
    }

    invalidFrontCount = 0;

    if (front <= TASK2_FRONT_STOP_MM)
    {
        beginSettle(reverseState);
        return;
    }

    float error = 0.0f;
    int correction = 0;

    if (tofSensors.leftValid() && tofSensors.rightValid())
    {
        // Positive error: closer to the right wall, so steer left.
        error = (float)(left - right);
        float derivative = error - previousWallError;
        previousWallError = error;

        correction = (int)(
            TASK2_WALL_KP * error +
            TASK2_WALL_KD * derivative
        );

        correction = constrain(
            correction,
            -TASK2_MAX_CORRECTION,
            TASK2_MAX_CORRECTION
        );
    }

    int leftSpeed = TASK2_BASE_SPEED - correction;
    int rightSpeed = TASK2_BASE_SPEED + correction;

    motor.setMotors(leftSpeed, rightSpeed);
    printToFDebug(error, correction);
}

void Task2Controller::updateRoomReverse(Task2State nextState)
{
    if (!tofSensors.update())
    {
        return;
    }

    unsigned long elapsed = millis() - stateStartTime;

    if (elapsed >= (unsigned long)TASK2_REVERSE_TIMEOUT_MS)
    {
        setError("Room reverse timeout; check exit distance");
        return;
    }

    int left = tofSensors.leftMM();
    int front = tofSensors.frontMM();
    int right = tofSensors.rightMM();

    bool frontAtEntrance =
        tofSensors.frontValid() &&
        front >= TASK2_ROOM_EXIT_FRONT_MM;

    bool wideJunctionDetected =
        sensors.activeCount() >= JUNCTION_TOTAL_MIN;

    if (elapsed >= (unsigned long)TASK2_REVERSE_MIN_MS &&
        (frontAtEntrance || wideJunctionDetected))
    {
        beginSettle(nextState);
        return;
    }

    float error = 0.0f;
    int correction = 0;

    if (tofSensors.leftValid() && tofSensors.rightValid())
    {
        error = (float)(left - right);
        float derivative = error - previousWallError;
        previousWallError = error;

        correction = (int)(
            TASK2_WALL_KP * error +
            TASK2_WALL_KD * derivative
        );

        correction = constrain(
            correction,
            -TASK2_MAX_CORRECTION,
            TASK2_MAX_CORRECTION
        );
    }

    // Reverse steering uses the opposite correction sign from forward motion.
    int leftSpeed = -TASK2_REVERSE_SPEED + correction;
    int rightSpeed = -TASK2_REVERSE_SPEED - correction;

    motor.setMotors(leftSpeed, rightSpeed);
    printToFDebug(error, correction);
}

void Task2Controller::followLineSimple()
{
    if (!sensors.lineDetected())
    {
        // Temporary safe behaviour between the room junctions.
        motor.stop();
        return;
    }

    float error = (float)(sensors.position() * POSITION_SIGN);
    float correction = pid.calculate(error);

    int leftSpeed = TASK2_LINE_SPEED - (int)correction;
    int rightSpeed = TASK2_LINE_SPEED + (int)correction;

    leftSpeed = constrain(leftSpeed, -PWM_MAX, PWM_MAX);
    rightSpeed = constrain(rightSpeed, -PWM_MAX, PWM_MAX);

    motor.setMotors(leftSpeed, rightSpeed);
}

void Task2Controller::updateFollowToJunction(
    Task2State nextState,
    unsigned long ignoreTimeMS
)
{
    unsigned long elapsed = millis() - stateStartTime;

    bool possibleJunction =
        elapsed >= ignoreTimeMS &&
        sensors.activeCount() >= JUNCTION_TOTAL_MIN;

    if (possibleJunction)
    {
        if (junctionCandidateTime == 0)
        {
            junctionCandidateTime = millis();
        }
        else if (millis() - junctionCandidateTime >=
                 (unsigned long)TASK2_JUNCTION_CONFIRM_MS)
        {
            beginSettle(nextState);
            return;
        }
    }
    else
    {
        junctionCandidateTime = 0;
    }

    followLineSimple();
}

void Task2Controller::setError(const char *message)
{
    motor.stop();
    currentState = TASK2_ERROR;

    Serial.print("[TASK2 ERROR] ");
    Serial.println(message);
}

void Task2Controller::printToFDebug(float error, int correction)
{
    if (millis() - lastDebugTime < (unsigned long)TASK2_DEBUG_RATE_MS)
    {
        return;
    }

    lastDebugTime = millis();

    Serial.print("[TASK2] L=");
    Serial.print(tofSensors.leftMM());
    Serial.print(" F=");
    Serial.print(tofSensors.frontMM());
    Serial.print(" R=");
    Serial.print(tofSensors.rightMM());
    Serial.print(" error=");
    Serial.print(error);
    Serial.print(" correction=");
    Serial.println(correction);
}

void Task2Controller::update()
{
    switch (currentState)
    {
        case TASK2_IDLE:
            motor.stop();
            break;

        case TASK2_FOLLOW_TO_JUNCTION_1:
            updateFollowToJunction(TASK2_TURN_LEFT_ROOM_1, 0);
            break;

        case TASK2_TURN_LEFT_ROOM_1:
            updateTimedTurn(false, TASK2_TURN_90_MS, TASK2_ENTER_ROOM_1);
            break;

        case TASK2_ENTER_ROOM_1:
            updateRoomForward(TASK2_REVERSE_ROOM_1);
            break;

        case TASK2_REVERSE_ROOM_1:
            updateRoomReverse(TASK2_TURN_180_TO_ROOM_2);
            break;

        case TASK2_TURN_180_TO_ROOM_2:
            updateTimedTurn(true, TASK2_TURN_180_MS, TASK2_ENTER_ROOM_2);
            break;

        case TASK2_ENTER_ROOM_2:
            updateRoomForward(TASK2_REVERSE_ROOM_2);
            break;

        case TASK2_REVERSE_ROOM_2:
            updateRoomReverse(TASK2_TURN_LEFT_TO_JUNCTION_2);
            break;

        case TASK2_TURN_LEFT_TO_JUNCTION_2:
            updateTimedTurn(false, TASK2_TURN_90_MS, TASK2_FOLLOW_TO_JUNCTION_2);
            break;

        case TASK2_FOLLOW_TO_JUNCTION_2:
            updateFollowToJunction(
                TASK2_TURN_LEFT_ROOM_3,
                TASK2_JUNCTION_IGNORE_MS
            );
            break;

        case TASK2_TURN_LEFT_ROOM_3:
            updateTimedTurn(false, TASK2_TURN_90_MS, TASK2_ENTER_ROOM_3);
            break;

        case TASK2_ENTER_ROOM_3:
            updateRoomForward(TASK2_REVERSE_ROOM_3);
            break;

        case TASK2_REVERSE_ROOM_3:
            updateRoomReverse(TASK2_TURN_180_TO_ROOM_4);
            break;

        case TASK2_TURN_180_TO_ROOM_4:
            updateTimedTurn(true, TASK2_TURN_180_MS, TASK2_ENTER_ROOM_4);
            break;

        case TASK2_ENTER_ROOM_4:
            updateRoomForward(TASK2_REVERSE_ROOM_4);
            break;

        case TASK2_REVERSE_ROOM_4:
            updateRoomReverse(TASK2_TURN_LEFT_TO_RAMP);
            break;

        case TASK2_TURN_LEFT_TO_RAMP:
            updateTimedTurn(false, TASK2_TURN_90_MS, TASK2_FOLLOW_TO_RAMP);
            break;

        case TASK2_FOLLOW_TO_RAMP:
            // Continue using the proven Task 1 line logic toward the ramp.
            track.update();
            break;

        case TASK2_SETTLE:
            updateSettle();
            break;

        case TASK2_ERROR:
            motor.stop();
            break;
    }
}

void Task2Controller::printState()
{
    Serial.print("[TASK2 STATE] ");

    switch (currentState)
    {
        case TASK2_IDLE: Serial.println("IDLE"); break;
        case TASK2_FOLLOW_TO_JUNCTION_1: Serial.println("FOLLOW LINE TO JUNCTION 1"); break;
        case TASK2_TURN_LEFT_ROOM_1: Serial.println("TURN LEFT TO ROOM 1"); break;
        case TASK2_ENTER_ROOM_1: Serial.println("ENTER ROOM 1"); break;
        case TASK2_REVERSE_ROOM_1: Serial.println("REVERSE FROM ROOM 1"); break;
        case TASK2_TURN_180_TO_ROOM_2: Serial.println("TURN 180 CLOCKWISE TO ROOM 2"); break;
        case TASK2_ENTER_ROOM_2: Serial.println("ENTER ROOM 2"); break;
        case TASK2_REVERSE_ROOM_2: Serial.println("REVERSE FROM ROOM 2"); break;
        case TASK2_TURN_LEFT_TO_JUNCTION_2: Serial.println("TURN LEFT TOWARD JUNCTION 2"); break;
        case TASK2_FOLLOW_TO_JUNCTION_2: Serial.println("FOLLOW LINE TO JUNCTION 2"); break;
        case TASK2_TURN_LEFT_ROOM_3: Serial.println("TURN LEFT TO ROOM 3"); break;
        case TASK2_ENTER_ROOM_3: Serial.println("ENTER ROOM 3"); break;
        case TASK2_REVERSE_ROOM_3: Serial.println("REVERSE FROM ROOM 3"); break;
        case TASK2_TURN_180_TO_ROOM_4: Serial.println("TURN 180 CLOCKWISE TO ROOM 4"); break;
        case TASK2_ENTER_ROOM_4: Serial.println("ENTER ROOM 4"); break;
        case TASK2_REVERSE_ROOM_4: Serial.println("REVERSE FROM ROOM 4"); break;
        case TASK2_TURN_LEFT_TO_RAMP: Serial.println("TURN LEFT TOWARD RAMP"); break;
        case TASK2_FOLLOW_TO_RAMP: Serial.println("FOLLOW LINE TO RAMP"); break;
        case TASK2_SETTLE: Serial.println("SETTLE"); break;
        case TASK2_ERROR: Serial.println("ERROR"); break;
    }
}
