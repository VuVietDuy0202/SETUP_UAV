#pragma once

#include <Arduino.h>
#include <Wire.h>

// ---------------------------------------------------------------------------
// Pin definitions
// ---------------------------------------------------------------------------
#define RED_LED_PIN 2
#define GREEN_LED_PIN 3

// ---------------------------------------------------------------------------
// MPU-6050
// ---------------------------------------------------------------------------
extern uint8_t gyro_address;

// ---------------------------------------------------------------------------
// Manual calibration values (defined in main.cpp)
// ---------------------------------------------------------------------------
extern int16_t manual_acc_pitch_cal_value;
extern int16_t manual_acc_roll_cal_value;
extern uint8_t use_manual_calibration;
extern int16_t manual_gyro_pitch_cal_value;
extern int16_t manual_gyro_roll_cal_value;
extern int16_t manual_gyro_yaw_cal_value;

// ---------------------------------------------------------------------------
// Sensor data
// ---------------------------------------------------------------------------
extern int16_t acc_axis[4];  // [1]=X  [2]=Y  [3]=Z  (index 0 unused)
extern int16_t gyro_axis[4]; // [1]=X  [2]=Y  [3]=Z
extern int16_t temperature;

extern int32_t gyro_axis_cal[4];
extern int32_t acc_axis_cal[4];

// ---------------------------------------------------------------------------
// Angle outputs
// ---------------------------------------------------------------------------
extern float angle_pitch;
extern float angle_roll;
extern float angle_pitch_acc;
extern float angle_roll_acc;

// ---------------------------------------------------------------------------
// RC channel values (updated every loop from ISR buffer)
// ---------------------------------------------------------------------------
extern int32_t channel_1; // Roll
extern int32_t channel_2; // Pitch
extern int32_t channel_3; // Throttle
extern int32_t channel_4; // Yaw
extern int32_t channel_5;
extern int32_t channel_6;

// ---------------------------------------------------------------------------
// General state
// ---------------------------------------------------------------------------
extern uint8_t data;
extern uint8_t start;
extern uint8_t warning;
extern uint8_t disable_throttle;
extern uint8_t flip32;
extern int16_t loop_counter;
extern uint32_t loop_timer;
extern int32_t cal_int;

// ---------------------------------------------------------------------------
// Functions defined in main.cpp
// ---------------------------------------------------------------------------
void print_intro(void);
void red_led(int8_t level);
void green_led(int8_t level);