#pragma once

#include <Arduino.h>

enum Task2State
{
    TASK2_IDLE,
    TASK2_FOLLOW_TO_JUNCTION_1,
    TASK2_TURN_LEFT_ROOM_1,
    TASK2_ENTER_ROOM_1,
    TASK2_REVERSE_ROOM_1,
    TASK2_TURN_180_TO_ROOM_2,
    TASK2_ENTER_ROOM_2,
    TASK2_REVERSE_ROOM_2,
    TASK2_TURN_LEFT_TO_JUNCTION_2,
    TASK2_FOLLOW_TO_JUNCTION_2,
    TASK2_TURN_LEFT_ROOM_3,
    TASK2_ENTER_ROOM_3,
    TASK2_REVERSE_ROOM_3,
    TASK2_TURN_180_TO_ROOM_4,
    TASK2_ENTER_ROOM_4,
    TASK2_REVERSE_ROOM_4,
    TASK2_TURN_LEFT_TO_RAMP,
    TASK2_FOLLOW_TO_RAMP,
    TASK2_SETTLE,
    TASK2_ERROR
};

class Task2Controller
{
public:
    void begin();
    bool start();
    void stop();
    void update();

    Task2State state() const;

private:
    void changeState(Task2State newState);
    void beginSettle(Task2State nextState);
    void updateSettle();

    void updateTimedTurn(
        bool clockwise,
        unsigned long turnTimeMS,
        Task2State nextState
    );

    void updateRoomForward(Task2State reverseState);
    void updateRoomReverse(Task2State nextState);

    void updateFollowToJunction(
        Task2State nextState,
        unsigned long ignoreTimeMS
    );
    void followLineSimple();

    void setError(const char *message);
    void printState();
    void printToFDebug(float error, int correction);

    Task2State currentState = TASK2_IDLE;
    Task2State stateAfterSettle = TASK2_IDLE;

    unsigned long stateStartTime = 0;
    unsigned long junctionCandidateTime = 0;
    unsigned long lastDebugTime = 0;

    float previousWallError = 0.0f;
    int invalidFrontCount = 0;
};

extern Task2Controller task2;
