#include "calib_mpu.h"

// Gyro scale: 500 dps  -> sensitivity = 65.5 LSB/(°/s)
// Loop rate : 250 Hz
// dt        = 1/250 = 0.004 s
// deg/LSB   = 1/65.5
// factor    = dt / sensitivity = 0.004 / 65.5 ≈ 0.0000611  <-- WRONG for 500 dps
// Correct   : 1 / (250 * 65.5) = 0.00006106  -- same value, so the constant IS correct
// Note: 65.5 LSB/(°/s) is the sensitivity for 500 dps (FS_SEL=1, register 0x08).
//       250 dps would give 131 LSB/(°/s) -> factor 0.0000305.
//       The value 0.0000611 is therefore correct for 500 dps at 250 Hz.
#define GYRO_SCALE_FACTOR 0.0000611f // 1 / (250 Hz * 65.5 LSB/dps)
#define GYRO_YAW_RAD 0.000001066f    // GYRO_SCALE_FACTOR * (PI/180)

void i2c_scanner(void)
{

    uint8_t error, address;
    uint16_t nDevices = 0;

    Serial.println("Scanning addresses 1 to 127...");
    Serial.println("");

    for (address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0)
        {
            Serial.print("I2C device found at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.println(address, HEX);
            nDevices++;
        }
        else if (error == 4)
        {
            Serial.print("Unknown error at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.println(address, HEX);
        }
    }

    if (nDevices == 0)
        Serial.println("No I2C devices found");
    else
        Serial.println("done");

    delay(2000);
}

void gyro_signalen(void)
{
    Wire.beginTransmission(gyro_address);
    Wire.write(0x3B); // Start at ACCEL_XOUT_H (register 0x3B)
    Wire.endTransmission();
    Wire.requestFrom(gyro_address, 14);

    // Accelerometer (X, Y, Z)
    acc_axis[1] = Wire.read() << 8 | Wire.read();
    acc_axis[2] = Wire.read() << 8 | Wire.read();
    acc_axis[3] = Wire.read() << 8 | Wire.read();

    // Temperature
    temperature = Wire.read() << 8 | Wire.read();

    // Gyroscope (X, Y, Z)
    gyro_axis[1] = Wire.read() << 8 | Wire.read();
    gyro_axis[2] = Wire.read() << 8 | Wire.read();
    gyro_axis[3] = Wire.read() << 8 | Wire.read();

    // Sign corrections so that nose-up = positive pitch, nose-right = positive yaw
    gyro_axis[2] *= -1;
    gyro_axis[3] *= -1;

    // Subtract manual calibration offsets
    acc_axis[1] -= manual_acc_pitch_cal_value;
    acc_axis[2] -= manual_acc_roll_cal_value;
    gyro_axis[1] -= manual_gyro_roll_cal_value;
    gyro_axis[2] -= manual_gyro_pitch_cal_value;
    gyro_axis[3] -= manual_gyro_yaw_cal_value;
}

void manual_imu_calibration(void)
{
    acc_axis_cal[1] = 0;
    acc_axis_cal[2] = 0;
    gyro_axis_cal[1] = 0;
    gyro_axis_cal[2] = 0;
    gyro_axis_cal[3] = 0;

    // --- Warm-up: 2000 readings discarded so the sensor stabilises ---
    Serial.print("Warming up sensor");
    for (cal_int = 0; cal_int < 2000; cal_int++)
    {
        if (cal_int % 250 == 0)
            Serial.print(".");
        gyro_signalen();
        delay(4); // ~250 Hz
    }

    // --- Calibration: 4000 readings averaged ---
    Serial.print("Calibrating");
    for (cal_int = 0; cal_int < 4000; cal_int++)
    {
        if (cal_int % 125 == 0)
        {
            digitalWrite(GREEN_LED_PIN, !digitalRead(GREEN_LED_PIN));
            Serial.print(".");
        }
        gyro_signalen();

        // Add back the manual offset so we measure the true raw value
        acc_axis_cal[1] += acc_axis[1] + manual_acc_pitch_cal_value;
        acc_axis_cal[2] += acc_axis[2] + manual_acc_roll_cal_value;
        gyro_axis_cal[1] += gyro_axis[1] + manual_gyro_roll_cal_value;
        gyro_axis_cal[2] += gyro_axis[2] + manual_gyro_pitch_cal_value;
        gyro_axis_cal[3] += gyro_axis[3] + manual_gyro_yaw_cal_value;

        delay(4);
    }
    Serial.println(".");

    // Average over 4000 samples
    acc_axis_cal[1] /= 4000;
    acc_axis_cal[2] /= 4000;
    gyro_axis_cal[1] /= 4000;
    gyro_axis_cal[2] /= 4000;
    gyro_axis_cal[3] /= 4000;

    Serial.print("manual_acc_pitch_cal_value = ");
    Serial.println(acc_axis_cal[1]);
    Serial.print("manual_acc_roll_cal_value = ");
    Serial.println(acc_axis_cal[2]);
    Serial.print("manual_gyro_pitch_cal_value = ");
    Serial.println(gyro_axis_cal[2]);
    Serial.print("manual_gyro_roll_cal_value = ");
    Serial.println(gyro_axis_cal[1]);
    Serial.print("manual_gyro_yaw_cal_value = ");
    Serial.println(gyro_axis_cal[3]);
}

void read_gyro_values(void)
{
    if (use_manual_calibration)
    {
        cal_int = 2000; // Treat manual values as already calibrated
    }
    else
    {
        cal_int = 0;
        manual_gyro_pitch_cal_value = 0;
        manual_gyro_roll_cal_value = 0;
        manual_gyro_yaw_cal_value = 0;
    }

    while (data != 'q')
    {
        delay(250);

        if (Serial.available() > 0)
        {
            data = Serial.read();
            delay(100);
            while (Serial.available() > 0)
                loop_counter = Serial.read(); // flush
        }

        // Run auto-calibration on first use or when 'c' is pressed
        if (data == 'c' && cal_int != 2000)
        {
            gyro_axis_cal[1] = 0;
            gyro_axis_cal[2] = 0;
            gyro_axis_cal[3] = 0;

            Serial.print("Calibrating the gyro");
            for (cal_int = 0; cal_int < 2000; cal_int++)
            {
                if (cal_int % 125 == 0)
                {
                    digitalWrite(GREEN_LED_PIN, !digitalRead(GREEN_LED_PIN));
                    Serial.print(".");
                }
                gyro_signalen();
                gyro_axis_cal[1] += gyro_axis[1];
                gyro_axis_cal[2] += gyro_axis[2];
                gyro_axis_cal[3] += gyro_axis[3];
                delay(4);
            }
            Serial.println(".");

            gyro_axis_cal[1] /= 2000;
            gyro_axis_cal[2] /= 2000;
            gyro_axis_cal[3] /= 2000;

            manual_gyro_pitch_cal_value = gyro_axis_cal[2];
            manual_gyro_roll_cal_value = gyro_axis_cal[1];
            manual_gyro_yaw_cal_value = gyro_axis_cal[3];
        }

        gyro_signalen();

        if (data == 'c')
        {
            Serial.print("Gyro_x = ");
            Serial.print(gyro_axis[1]);
            Serial.print(" Gyro_y = ");
            Serial.print(gyro_axis[2]);
            Serial.print(" Gyro_z = ");
            Serial.println(gyro_axis[3]);
        }
        else
        {
            Serial.print("ACC_x = ");
            Serial.print(acc_axis[1]);
            Serial.print(" ACC_y = ");
            Serial.print(acc_axis[2]);
            Serial.print(" ACC_z = ");
            Serial.println(acc_axis[3]);
        }
    }
}

void check_imu_angles(void)
{
    bool first_angle = true;
    loop_counter = 0;

    if (use_manual_calibration)
    {
        cal_int = 2000;
    }
    else
    {
        cal_int = 0;
        manual_gyro_pitch_cal_value = 0;
        manual_gyro_roll_cal_value = 0;
        manual_gyro_yaw_cal_value = 0;
    }

    // Auto-calibrate if no manual calibration values are available
    if (cal_int == 0)
    {
        gyro_axis_cal[1] = 0;
        gyro_axis_cal[2] = 0;
        gyro_axis_cal[3] = 0;

        Serial.print("Calibrating the gyro");
        for (cal_int = 0; cal_int < 2000; cal_int++)
        {
            if (cal_int % 125 == 0)
            {
                digitalWrite(GREEN_LED_PIN, !digitalRead(GREEN_LED_PIN));
                Serial.print(".");
            }
            gyro_signalen();
            gyro_axis_cal[1] += gyro_axis[1];
            gyro_axis_cal[2] += gyro_axis[2];
            gyro_axis_cal[3] += gyro_axis[3];
            delay(4);
        }
        Serial.println(".");

        gyro_axis_cal[1] /= 2000;
        gyro_axis_cal[2] /= 2000;
        gyro_axis_cal[3] /= 2000;

        manual_gyro_pitch_cal_value = gyro_axis_cal[2];
        manual_gyro_roll_cal_value = gyro_axis_cal[1];
        manual_gyro_yaw_cal_value = gyro_axis_cal[3];
    }

    while (data != 'q')
    {
        loop_timer = micros() + 4000; // 250 Hz loop

        if (Serial.available() > 0)
        {
            data = Serial.read();
            delay(100);
            while (Serial.available() > 0)
                loop_counter = Serial.read();
        }

        gyro_signalen();

        // --- Gyro angle integration (250 Hz, 500 dps scale) ---
        angle_pitch += gyro_axis[2] * GYRO_SCALE_FACTOR;
        angle_roll += gyro_axis[1] * GYRO_SCALE_FACTOR;

        // Transfer between axes when yawing
        angle_pitch -= angle_roll * sin(gyro_axis[3] * GYRO_YAW_RAD);
        angle_roll += angle_pitch * sin(gyro_axis[3] * GYRO_YAW_RAD);

        // --- Accelerometer angles ---
        // Clamp to ±8g range (4096 LSB/g at ±8g)
        acc_axis[1] = constrain(acc_axis[1], -4096, 4096);
        acc_axis[2] = constrain(acc_axis[2], -4096, 4096);

        // 57.296 = 180/PI  (asin returns radians)
        angle_pitch_acc = asin((float)acc_axis[1] / 4096.0f) * 57.296f;
        angle_roll_acc = asin((float)acc_axis[2] / 4096.0f) * 57.296f;

        // --- Complementary filter ---
        if (first_angle)
        {
            angle_pitch = angle_pitch_acc;
            angle_roll = angle_roll_acc;
            first_angle = false;
        }
        else
        {
            angle_pitch = angle_pitch * 0.9996f + angle_pitch_acc * 0.0004f;
            angle_roll = angle_roll * 0.9996f + angle_roll_acc * 0.0004f;
        }

        // --- Serial output spread over 60 loop cycles to avoid blocking ---
        switch (loop_counter)
        {
        case 0:
            Serial.print("Pitch: ");
            break;
        case 1:
            Serial.print(angle_pitch, 1);
            break;
        case 2:
            Serial.print(" Roll: ");
            break;
        case 3:
            Serial.print(angle_roll, 1);
            break;
        case 4:
            Serial.print(" Yaw: ");
            break;
        case 5:
            Serial.print(gyro_axis[3] / 65.5f, 0);
            break;
        case 6:
            Serial.print(" Temp: ");
            break;
        case 7:
            Serial.println(temperature / 340.0f + 36.53f, 1);
            break;
        default:
            break;
        }

        loop_counter++;
        if (loop_counter == 60)
            loop_counter = 0;

        while (loop_timer > micros())
            ; // busy-wait to maintain 250 Hz
    }

    loop_counter = 0;
    // print_intro();
}