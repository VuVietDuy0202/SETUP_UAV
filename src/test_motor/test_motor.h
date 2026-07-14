#pragma once

#include "common.h"
#include "calib_mpu/calib_mpu.h" // gyro_signalen()
#include "receiver/receiver.h"   // getReceiverValues()
#include <ESP32Servo.h>

#define MOT1_PIN 4
#define MOT2_PIN 5
#define MOT3_PIN 6
#define MOT4_PIN 7

extern Servo mot1;
extern Servo mot2;
extern Servo mot3;
extern Servo mot4;

// Attach ESCs and arm at 1000 us. Call once inside setup().
void motors_init(void);

// Interactive motor vibration test. Quit with 'q'.
void check_motor_vibrations(void);