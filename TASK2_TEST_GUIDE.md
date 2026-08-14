# Task 2 Wall-Following Test

## Hardware

- ESP32-C6 GPIO4 -> TCA9548A SDA
- ESP32-C6 GPIO5 -> TCA9548A SCL
- ESP32-C6 GPIO10 -> one side of the task-select push button
- Other side of the push button -> GND
- TCA channel 0 -> left VL53L0X
- TCA channel 1 -> front VL53L0X
- TCA channel 2 -> right VL53L0X
- Use 3.3 V and a common GND

The button uses `INPUT_PULLUP`; no external button resistor is required.

## Starting Task 2 for testing

1. Place the robot on the line before the first room junction, facing toward it.
2. Complete the normal line-sensor calibration.
3. Press the GPIO10 button to change from Task 1 to Task 2.

Alternative: hold the button while resetting/powering the ESP32. The selection
is recorded immediately, so the button can be released during calibration.

## Programmed route

1. Follow the line to Junction 1.
2. Turn 90 degrees left and enter Room 1 using left/right ToF centering.
3. Stop near the front wall/box and reverse to Junction 1.
4. Turn 180 degrees clockwise and enter Room 2.
5. Reverse to Junction 1, turn 90 degrees left, and follow the line to Junction 2.
6. Repeat the same sequence for Rooms 3 and 4.
7. Turn 90 degrees left and continue following the line toward the ramp.

## First constants to calibrate

All constants are in `include/Config.h`.

- `TASK2_TURN_90_MS`: tune until the robot turns exactly 90 degrees.
- `TASK2_TURN_180_MS`: tune until the robot turns exactly 180 degrees.
- `TASK2_FRONT_STOP_MM`: front wall/box stopping clearance.
- `TASK2_ROOM_EXIT_FRONT_MM`: front reading when the robot is back at the doorway.
- `TASK2_WALL_KP` and `TASK2_WALL_KD`: wall-centering response.
- `TOF_LEFT_OFFSET_MM`, `TOF_FRONT_OFFSET_MM`, and `TOF_RIGHT_OFFSET_MM`: measured sensor offsets.

Robot width is 210 mm and room width is 300 mm, so the nominal physical side
clearance is 45 mm per side. The controller uses `left - right`, so the two side
readings should be approximately equal when the robot is centered.

## Safe first test

Lift the wheels off the floor first and check the motor directions in Serial
Monitor at 115200 baud. Then test at low speed with a hand ready at the power
switch. The robot stops after three invalid front readings or after a reverse
timeout.
