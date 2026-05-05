#pragma once

#include "common.h"

// ---------------------------------------------------------------------------
// MPU-6050 calibration and angle reading functions
// ---------------------------------------------------------------------------

// Scan all I2C addresses and print found devices to Serial
void i2c_scanner(void);

// Read raw accelerometer, gyro, and temperature from MPU-6050.
// Results stored in acc_axis[], gyro_axis[], temperature globals.
// Manual calibration offsets are applied automatically.
void gyro_signalen(void);

// Collect 4000 samples (after a 2000-sample warm-up) and print
// the average calibration values to Serial.
// Place the IMU flat and still before calling this.
void manual_imu_calibration(void);

// Interactive loop: display raw gyro ('c') or accelerometer values.
// Runs until 'q' is received over Serial.
void read_gyro_values(void);

// Interactive loop: compute and display pitch / roll / yaw angles
// using a complementary filter. Runs until 'q' is received over Serial.
void check_imu_angles(void);