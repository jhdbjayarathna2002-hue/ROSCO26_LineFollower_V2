// ============================================================
// LineFollower.cpp  —  High-level robot state machine
// ============================================================
// See LineFollower.h for full documentation.
// ============================================================

#include "LineFollower.h"

// ============================================================
// Construction
// ============================================================

LineFollower::LineFollower()
    : _robotState(ROBOT_STARTUP),
      _lastLoopUs(0),
      _lastDebugMs(0),
      _lineLostStartMs(0),
      _searchStartMs(0),
      _targetSpeed(BASE_SPEED),
      _currentSpeed(0),
      _inTurn(false),
      _turnDirection(0),
      _inJunction(false),
      _shortLineLoss(false)
{
}

// ============================================================
// begin()  —  one-time initialisation from setup()
// ============================================================

void LineFollower::begin()
{
    _robotState = ROBOT_STARTUP;

    // Initialise subsystems
    sensor.begin();
    motor.begin();     // STBY LOW, motors stopped
    pid.reset();

    Serial.println(F("============================================"));
    Serial.println(F(" ROSCO'26  Task 01  Line Follower"));
    Serial.println(F("============================================"));

#if LINE_IS_WHITE
    Serial.println(F(" Track colour: WHITE LINE on BLACK (ROSCO'26 official)"));
#else
    Serial.println(F(" Track colour: BLACK LINE on WHITE (lab testing)"));
#endif

    Serial.println(F("--------------------------------------------"));

    _robotState = ROBOT_CALIBRATING;

#if DEVELOPMENT_MODE
    runDevelopmentCalibration();
#else
    runCompetitionCalibration();
#endif
}

// ============================================================
// runDevelopmentCalibration()
// Called once from begin().  Auto white+black calibration.
// ============================================================

void LineFollower::runDevelopmentCalibration()
{
    Serial.println(F(" [DEV MODE]  Automatic calibration starting..."));
    Serial.println(F(" Motor driver DISABLED (STBY LOW) during calibration."));
    motor.disable();   // STBY LOW for safety

    bool ok = sensor.calibrate();

    if (!ok)
    {
        enterFault("Calibration FAILED — one or more sensors invalid.");
        return;
    }

    // Save calibration for later competition use
    sensor.saveCalibration();

    Serial.println(F(" Calibration complete.  Resetting PID."));
    pid.reset();

    Serial.println(F(" Enabling motor driver (STBY HIGH)."));
    motor.enable();    // STBY HIGH — ready to move

    _robotState  = ROBOT_RUNNING;
    _lastLoopUs  = micros();
    _lastDebugMs = millis();

    Serial.println(F(" Robot entering RUNNING state."));
    Serial.println(F("============================================"));
}

// ============================================================
// runCompetitionCalibration()
// COMPETITION MODE: loads stored calibration from NVS.
// No program changes or hardware replacement allowed.
// The stored values were saved during the 2-minute calibration
// period that precedes gameplay.
// ============================================================

void LineFollower::runCompetitionCalibration()
{
    Serial.println(F(" [COMPETITION MODE]  Loading stored calibration..."));
    motor.disable();   // STBY LOW during load

    bool ok = sensor.loadCalibration();

    if (!ok)
    {
        // Stored calibration not found — fall back to auto-calibrate once
        Serial.println(F(" No stored calibration found."));
        Serial.println(F(" Running auto-calibration and saving..."));

        ok = sensor.calibrate();
        if (!ok)
        {
            enterFault("Calibration FAILED — cannot start competition mode.");
            return;
        }
        sensor.saveCalibration();
    }

    pid.reset();
    motor.enable();

    _robotState  = ROBOT_RUNNING;
    _lastLoopUs  = micros();
    _lastDebugMs = millis();

    Serial.println(F(" Robot entering RUNNING state (COMPETITION)."));
    Serial.println(F("============================================"));
}

// ============================================================
// update()  —  call from loop(), non-blocking
// ============================================================

void LineFollower::update()
{
    if (_robotState == ROBOT_FAULT)
    {
        handleFault();
        return;
    }

    if (_robotState != ROBOT_RUNNING)
        return;

#if SYSTEM_MODE == MODE_MOTOR_TEST
    runMotorTest();
    return;
#elif SYSTEM_MODE == MODE_SENSOR_TEST
    runSensorTest();
    return;
#endif

    // ---- Enforce minimum control loop period ----------------
    unsigned long nowUs = micros();
    if ((nowUs - _lastLoopUs) < (unsigned long)CONTROL_LOOP_US)
        return;

    // Calculate dt ONCE per control loop iteration
    float dt = (float)(nowUs - _lastLoopUs) / 1000000.0f;
    if (dt <= 0.0f) dt = 0.001f;
    _lastLoopUs = nowUs;

#if SYSTEM_MODE == MODE_BASIC_PID_TEST
    runBasicPidTest(dt);
#else
    runControl(dt);
#endif

    // ---- Non-blocking debug print (much slower than loop) ---
#if defined(DEBUG_SENSOR) || defined(DEBUG_PID) || \
    defined(DEBUG_MOTOR)  || defined(DEBUG_TRACK) || \
    defined(DEBUG_PATTERN)
    if ((millis() - _lastDebugMs) >= (unsigned long)DEBUG_PRINT_MS)
    {
        _lastDebugMs = millis();
        printDebug();
    }
#endif
}

// ============================================================
// runControl()  —  one full control iteration
// ============================================================

void LineFollower::runControl(float dt)
{
    // 1. Read sensors
    sensor.update();

    // 2. Update track detector
    TrackState ts = detector.update(sensor);

    // 3. Debug state transitions
#ifdef DEBUG_STATE_CHANGE
    static TrackState prevTs = STATE_NORMAL_LINE;
    if (ts != prevTs)
    {
        Serial.printf("[STATE] %s -> %s | pattern=0x%04X pos=%d active=%d\n",
                      trackStateName(prevTs), trackStateName(ts),
                      sensor.getPattern(), sensor.lastPosition, sensor.activeSensorCount);
        prevTs = ts;
    }
#endif

    // 4. Dispatch to state handler using the passed-in dt
    switch (ts)
    {
        case STATE_NORMAL_LINE:  handleNormalLine(dt); break;
        case STATE_CURVE:        handleCurve(dt);      break;
        case STATE_TURN_LEFT:
        case STATE_TURN_RIGHT:   handleTurn(dt);       break;
        case STATE_JUNCTION:     handleJunction(dt);   break;
        case STATE_CIRCLE:       handleCircle(dt);     break;
        case STATE_DASHED_LINE:  handleNormalLine(dt); break;  // treat as normal
        case STATE_LINE_LOST:    handleLineLost(dt);   break;
        case STATE_SEARCHING:    handleSearching(dt);  break;
        case STATE_RECOVERY:     handleNormalLine(dt); break;
        case STATE_DEAD_END:     handleDeadEnd(dt);    break;
        case STATE_FAULT:        enterFault("TrackDetector FAULT"); break;
        default:                 handleNormalLine(dt); break;
    }
}

// ============================================================
// Diagnostic Mode: runMotorTest()
// Direct motor driver test — bypasses sensors, PID, and TrackDetector.
// ============================================================

void LineFollower::runMotorTest()
{
    static int testStep = 0;
    static unsigned long lastStepMs = 0;
    unsigned long now = millis();

    motor.enable();

    if (now - lastStepMs < 1500) return;
    lastStepMs = now;

    switch (testStep % 7)
    {
        case 0:
            Serial.println(F("[MOTOR TEST] Left FWD 50"));
            motor.setLeftMotor(50);
            motor.setRightMotor(0);
            break;
        case 1:
            Serial.println(F("[MOTOR TEST] Left REV 50"));
            motor.setLeftMotor(-50);
            motor.setRightMotor(0);
            break;
        case 2:
            Serial.println(F("[MOTOR TEST] Right FWD 50"));
            motor.setLeftMotor(0);
            motor.setRightMotor(50);
            break;
        case 3:
            Serial.println(F("[MOTOR TEST] Right REV 50"));
            motor.setLeftMotor(0);
            motor.setRightMotor(-50);
            break;
        case 4:
            Serial.println(F("[MOTOR TEST] Both FWD 50"));
            motor.setMotors(50, 50);
            break;
        case 5:
            Serial.println(F("[MOTOR TEST] Both Stop (Coast)"));
            motor.stop();
            break;
        case 6:
            Serial.println(F("[MOTOR TEST] Both Active Brake"));
            motor.brake();
            break;
    }
    testStep++;
}

// ============================================================
// Diagnostic Mode: runSensorTest()
// Sensor scanning test — motors stay disabled.
// ============================================================

void LineFollower::runSensorTest()
{
    motor.disable();
    sensor.update();

    static unsigned long lastPrintMs = 0;
    if (millis() - lastPrintMs >= SENSOR_TEST_DELAY_MS)
    {
        lastPrintMs = millis();
        sensor.printSensorTest();
    }
}

// ============================================================
// Diagnostic Mode: runBasicPidTest()
// Basic PID line following — bypasses turn/junction/circle state overrides.
// ============================================================

void LineFollower::runBasicPidTest(float dt)
{
    sensor.update();
    if (!sensor.lineDetected())
    {
        motor.stop();
        return;
    }

    int pos          = sensor.getPosition();
    float error      = (float)pos;
    float correction = pid.compute(error, dt);

    applyMotors(BASE_SPEED, correction);
}

// ============================================================
// handleNormalLine()
// Standard PID line-following on a straight or slight curve.
// ============================================================

void LineFollower::handleNormalLine(float dt)
{
    _shortLineLoss  = false;
    _searchStartMs  = 0;
    _inTurn         = false;

    int pos      = sensor.getPosition();
    float error  = (float)pos;

    float correction = pid.compute(error, dt);

    // Speed profile: faster when position error is small
    int absErr = abs(pos);
    if (absErr < 500)
        rampSpeed(STRAIGHT_SPEED);
    else
        rampSpeed(BASE_SPEED);

    applyMotors(_currentSpeed, correction);
}

// ============================================================
// handleCurve()
// Reduce speed; continue with PID.
// ============================================================

void LineFollower::handleCurve(float dt)
{
    int pos     = sensor.getPosition();
    float error = (float)pos;

    float correction = pid.compute(error, dt);

    rampSpeed(CURVE_SPEED);
    applyMotors(_currentSpeed, correction);
}

// ============================================================
// handleTurn()
// 90-degree hard turn: sensor-guided.
// 1. Slow down.
// 2. Identify direction (from TrackDetector state).
// 3. Spin until sensors re-centre on new line.
// ============================================================

void LineFollower::handleTurn(float dt)
{
    TrackState ts = detector.getState();

    if (!_inTurn)
    {
        _inTurn        = true;
        _turnDirection = (ts == STATE_TURN_LEFT) ? -1 : +1;
        pid.reset();
        Serial.printf("[TURN] Starting %s turn\n",
                      _turnDirection < 0 ? "LEFT" : "RIGHT");
    }

    // During turn: spin one motor forward, other reverse
    int turnSpd = TURN_SPEED;

    if (_turnDirection < 0)
    {
        // Left turn: left motor slow/reverse, right motor forward
        motor.setLeftMotor(-turnSpd / 2);
        motor.setRightMotor(turnSpd);
    }
    else
    {
        // Right turn: right motor slow/reverse, left motor forward
        motor.setLeftMotor(turnSpd);
        motor.setRightMotor(-turnSpd / 2);
    }

    // Check for line re-acquisition on centre sensors
    int centreCount = sensor.countActive(CENTER_GROUP_START, CENTER_GROUP_END);
    if (centreCount >= 2 && detector.stateAge() > 150)
    {
        // Line reacquired on centre — exit turn
        _inTurn = false;
        pid.reset();
        Serial.println(F("[TURN] Line reacquired. Exiting turn."));
    }
}

// ============================================================
// handleJunction()
// At a junction, select the correct branch.
// JUNCTION_DEFAULT_DIRECTION: 0=STRAIGHT, 1=LEFT, 2=RIGHT
// ============================================================

void LineFollower::handleJunction(float dt)
{
    if (!_inJunction)
    {
        _inJunction = true;
        pid.reset();

        JunctionType jt = detector.getJunctionType();

        Serial.printf("[JUNCTION] Type=%d  Default=%d\n",
                      (int)jt, JUNCTION_DEFAULT_DIRECTION);
    }

    rampSpeed(JUNCTION_SPEED);

    // Follow the configured default direction using position
#if JUNCTION_DEFAULT_DIRECTION == 1
    // Prefer LEFT branch
    int pos     = sensor.getPosition();
    float error = (float)pos + 3000.0f;  // bias toward left
#elif JUNCTION_DEFAULT_DIRECTION == 2
    // Prefer RIGHT branch
    int pos     = sensor.getPosition();
    float error = (float)pos - 3000.0f;  // bias toward right
#else
    // STRAIGHT — follow centre
    int pos     = sensor.getPosition();
    float error = (float)pos;
#endif

    float correction = pid.compute(error, dt);
    applyMotors(_currentSpeed, correction);

    // Exit junction when back to narrow line
    if (detector.getState() != STATE_JUNCTION)
    {
        _inJunction = false;
        pid.reset();
    }
}

// ============================================================
// handleCircle()
// Wide line / circular path: maintain speed and follow centre.
// ============================================================

void LineFollower::handleCircle(float dt)
{
    int pos     = sensor.getPosition();
    float error = (float)pos;

    float correction = pid.compute(error, dt);

    rampSpeed(CURVE_SPEED);
    applyMotors(_currentSpeed, correction);
}

// ============================================================
// handleLineLost()
// Two-stage approach:
//   1) Short loss  (dashed line): hold position, wait
//   2) Long loss: enter SEARCHING
// ============================================================

void LineFollower::handleLineLost(float dt)
{
    unsigned long now = millis();

    if (!_shortLineLoss)
    {
        _shortLineLoss   = true;
        _lineLostStartMs = now;
    }

    unsigned long lostDuration = now - _lineLostStartMs;

    if (lostDuration < (unsigned long)SHORT_LINE_LOSS_MS)
    {
        // Dashed-line gap: continue straight with last PID correction
        // Do not update PID with stale position; just hold last motor output
        applyMotors(_currentSpeed, pid.getLastOutput());
        return;
    }

    // Genuine line loss — transition to SEARCHING
    _shortLineLoss = false;
    if (_searchStartMs == 0)
    {
        _searchStartMs = now;
    }

    handleSearching(dt);
}

// ============================================================
// handleSearching()
// Spin toward the last known line side.
// Fix: Stable search start timestamp, avoids constant resetting.
// ============================================================

void LineFollower::handleSearching(float dt)
{
    unsigned long now = millis();
    if (_searchStartMs == 0)
    {
        _searchStartMs = now;
    }

    unsigned long searchAge = now - _searchStartMs;

    if (searchAge > (unsigned long)SEARCH_TIMEOUT_MS)
    {
        enterFault("Search timeout — line not recovered.");
        return;
    }

    // If line is found, reacquire
    if (sensor.lineDetected())
    {
        Serial.println(F("[SEARCH] Line reacquired. Resuming."));
        _shortLineLoss = false;
        _searchStartMs = 0;
        _inTurn        = false;
        pid.reset();
        return;
    }

    // Spin toward the side where the line was last seen
    int dir = (sensor.lastPosition < 0) ? -1 : +1;

    if (dir < 0)
    {
        // Search LEFT
        motor.setLeftMotor(-SEARCH_SPEED / 2);
        motor.setRightMotor(SEARCH_SPEED);
    }
    else
    {
        // Search RIGHT
        motor.setLeftMotor(SEARCH_SPEED);
        motor.setRightMotor(-SEARCH_SPEED / 2);
    }
}

// ============================================================
// handleDeadEnd()
// Sustained line loss after tracking line. Response is configurable;
// stops and enters fault safely.
// ============================================================

void LineFollower::handleDeadEnd(float dt)
{
    motor.brake();
    motor.disable();

    Serial.println(F("[DEAD_END] Dead end detected. Robot stopped safely."));
    _robotState = ROBOT_FAULT;
}

// ============================================================
// handleFault()
// Disable motors, keep STBY low, print periodic reminder.
// ============================================================

void LineFollower::handleFault()
{
    motor.disable();   // ensures STBY LOW

    // Print once per second so Serial does not flood
    if ((millis() - _lastDebugMs) >= 1000)
    {
        _lastDebugMs = millis();
        Serial.println(F("[FAULT] Robot halted. Reset to recover."));
    }
}

// ============================================================
// enterFault()
// ============================================================

void LineFollower::enterFault(const char* reason)
{
    Serial.print(F("[FAULT] "));
    Serial.println(reason);

    motor.brake();
    delay(50);
    motor.disable();

    _robotState = ROBOT_FAULT;
}

// ============================================================
// applyMotors()
// Apply PID correction to base speed, constrain, set motors.
// ============================================================

void LineFollower::applyMotors(int baseSpeed, float correction)
{
    int leftSpeed  = baseSpeed - (int)correction;
    int rightSpeed = baseSpeed + (int)correction;

    leftSpeed  = constrain(leftSpeed,  -PWM_MAX, PWM_MAX);
    rightSpeed = constrain(rightSpeed, -PWM_MAX, PWM_MAX);

#ifdef DEBUG_MOTOR_COMMANDS
    static int prevL = -999, prevR = -999;
    if (leftSpeed != prevL || rightSpeed != prevR)
    {
        Serial.printf("[MOTOR] STBY=%s  L=%d  R=%d  base=%d  corr=%.2f\n",
                      motor.isEnabled() ? "HIGH" : "LOW",
                      leftSpeed, rightSpeed, baseSpeed, correction);
        prevL = leftSpeed;
        prevR = rightSpeed;
    }
#endif

    motor.setMotors(leftSpeed, rightSpeed);
}

// ============================================================
// rampSpeed()
// Smooth acceleration / deceleration toward target speed.
// ============================================================

void LineFollower::rampSpeed(int target)
{
    _targetSpeed = target;

    if (_currentSpeed < _targetSpeed)
        _currentSpeed = min(_currentSpeed + SPEED_RAMP_STEP, _targetSpeed);
    else if (_currentSpeed > _targetSpeed)
        _currentSpeed = max(_currentSpeed - SPEED_RAMP_STEP, _targetSpeed);
}

// ============================================================
// printDebug()
// Non-blocking; called at DEBUG_PRINT_MS interval.
// ============================================================

void LineFollower::printDebug()
{
#ifdef DEBUG_SENSOR
    sensor.printLineStrength();
#endif

#ifdef DEBUG_PATTERN
    sensor.printPattern();
#endif

#ifdef DEBUG_PID
    Serial.printf("[PID] pos=%d  P=%.2f  I=%.2f  D=%.2f  out=%.2f  L=%d  R=%d\n",
                  sensor.lastPosition,
                  pid.getP(), pid.getI(), pid.getD(),
                  pid.getLastOutput(),
                  _currentSpeed - (int)pid.getLastOutput(),
                  _currentSpeed + (int)pid.getLastOutput());
#endif

#ifdef DEBUG_TRACK
    Serial.printf("[TRACK] state=%s  pattern=0x%04X  pos=%d  active=%d\n",
                  detector.stateName(),
                  sensor.getPattern(),
                  sensor.lastPosition,
                  sensor.activeSensorCount);
#endif

#ifdef DEBUG_MOTOR
    // Motor state is already printed inside Motor::setMotors
#endif
}
