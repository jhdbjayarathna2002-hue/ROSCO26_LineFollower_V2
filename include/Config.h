#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

/*=========================================================
  ROSCO'26 Line Follower
  Board  : ESP32-C6-WROOM-1
  Driver : L298N
  Sensor : 8 Analog IR Sensors
=========================================================*/


//=========================================================
// SENSOR CONFIGURATION
//=========================================================

#define SENSOR_COUNT 8

// Analog sensor pins
constexpr uint8_t SENSOR_PINS[SENSOR_COUNT] =
{
    0, 1, 2, 3,
    4, 5, 6, 7
};


//=========================================================
// MOTOR DRIVER (L298N)
//=========================================================

// Left Motor
#define ENA 18
#define IN1 19
#define IN2 20

// Right Motor
#define ENB 23
#define IN3 21
#define IN4 22


//=========================================================
// PWM
//=========================================================

#define PWM_FREQUENCY 1000
#define PWM_RESOLUTION 8

#define LEFT_PWM_CHANNEL 0
#define RIGHT_PWM_CHANNEL 1


//=========================================================
// ROBOT SPEED
//=========================================================

#define BASE_SPEED 120
#define MAX_SPEED 255
#define MIN_SPEED 0
#define SEARCH_SPEED 80        // speed for turning when line is lost


//=========================================================
// PID
//=========================================================

#define KP 0.10f
#define KI 0.00f
#define KD 0.00f


//=========================================================
// SENSOR
//=========================================================

#define CALIBRATION_SAMPLES 100    // readings per sensor per surface
#define CALIBRATION_COUNTDOWN 3000 // ms to blink before sampling

// Normalized value above this means
// the sensor is on the WHITE line.
#define LINE_THRESHOLD 500


//=========================================================
// BUILT-IN LED
//=========================================================

#define LED_PIN 8              // ESP32-C6 NeoPixel (GPIO8)
#define LED_BRIGHTNESS 30      // NeoPixel brightness (0-255)


//=========================================================
// DEBUG
//=========================================================

#define SERIAL_BAUD 115200

#endif