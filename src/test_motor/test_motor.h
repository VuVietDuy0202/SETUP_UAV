#pragma once

#include "common.h"
#include "calib_mpu/calib_mpu.h" // gyro_signalen()
#include "receiver/receiver.h"   // getReceiverValues()
#include <ESP32Servo.h>

// ---------------------------------------------------------------------------
// ESC / motor objects  (defined in test_motor.cpp)
// ---------------------------------------------------------------------------
//
// Motor layout (top view, X-frame):
//
//          Front
//   M1 (CCW)     M4 (CW)
//       \           /
//        [  frame  ]
//       /           \
//   M2 (CW)      M3 (CCW)
//          Rear
//
// Servo output pins -- adjust to match your ESC wiring:
#define MOT1_PIN 17
#define MOT2_PIN 18
#define MOT3_PIN 8
#define MOT4_PIN 3

extern Servo mot1;
extern Servo mot2;
extern Servo mot3;
extern Servo mot4;

// Attach ESCs and arm at 1000 us. Call once inside setup().
void motors_init(void);

// Interactive motor vibration test. Quit with 'q'.
void check_motor_vibrations(void);