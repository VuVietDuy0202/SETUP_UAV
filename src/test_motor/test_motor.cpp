#include "test_motor.h"

#define NUM_CHANNELS 6
#define VIB_SAMPLES 17 // window size (indices 0..16)

// ---------------------------------------------------------------------------
// Servo (ESC) objects -- defined here, declared extern in test_motor.h
// ---------------------------------------------------------------------------
Servo mot1;
Servo mot2;
Servo mot3;
Servo mot4;

// ---------------------------------------------------------------------------
// Attach ESCs and send 1000 us arm signal
// ---------------------------------------------------------------------------
void motors_init(void)
{
    mot1.attach(MOT1_PIN, 1000, 2000);
    mot2.attach(MOT2_PIN, 1000, 2000);
    mot3.attach(MOT3_PIN, 1000, 2000);
    mot4.attach(MOT4_PIN, 1000, 2000);

    mot1.writeMicroseconds(1000);
    mot2.writeMicroseconds(1000);
    mot3.writeMicroseconds(1000);
    mot4.writeMicroseconds(1000);

    delay(3000); // allow ESCs to complete arming sequence
    Serial.println(F("Motors initialised."));
}

void check_motor_vibrations(void)
{
    // FIX: array sized to match actual usage (indices 0..16 = 17 elements)
    int32_t vibration_array[VIB_SAMPLES] = {0};
    int32_t average_vibration_level = 0;
    int32_t vibration_total_result = 0;
    uint8_t array_counter = 0;
    uint8_t throttle_init_ok = 0;
    uint8_t vibration_counter = 0;
    uint32_t wait_timer;

    Serial.println(F("=== Motor Vibration Check ==="));
    Serial.println(F("Commands: 1=M1, 2=M2, 3=M3, 4=M4, 5=All, q=Quit"));

    while (data != 'q')
    {
        loop_timer = micros() + 4000; // 250 Hz

        if (Serial.available() > 0)
        {
            data = Serial.read();
            delay(100);
            while (Serial.available() > 0)
                loop_counter = Serial.read(); // flush
        }

        uint16_t ch[NUM_CHANNELS];
        getReceiverValues(ch);

        // FIX: use a local name that does not shadow the global int32_t channel_3
        uint16_t throttle_us = ch[2];

        if (throttle_init_ok)
        {
            gyro_signalen();

            // Compute vibration magnitude from accelerometer vector length
            vibration_array[0] = (int32_t)sqrt(
                (float)acc_axis[1] * acc_axis[1] +
                (float)acc_axis[2] * acc_axis[2] +
                (float)acc_axis[3] * acc_axis[3]);

            // Shift window and accumulate average
            average_vibration_level = 0;
            for (array_counter = VIB_SAMPLES - 1; array_counter > 0; array_counter--)
            {
                vibration_array[array_counter] = vibration_array[array_counter - 1];
                average_vibration_level += vibration_array[array_counter];
            }
            average_vibration_level /= (VIB_SAMPLES - 1); // average of the 16 history slots

            if (vibration_counter < 20)
            {
                vibration_counter++;
                vibration_total_result += abs(vibration_array[0] - average_vibration_level);
            }
            else
            {
                vibration_counter = 0;
                Serial.println(vibration_total_result / 50);
                vibration_total_result = 0;
            }

            uint16_t esc = constrain(throttle_us, 1000, 2000);

            if (data == '1')
            {
                mot1.writeMicroseconds(esc);
                mot2.writeMicroseconds(1000);
                mot3.writeMicroseconds(1000);
                mot4.writeMicroseconds(1000);
            }
            else if (data == '2')
            {
                mot1.writeMicroseconds(1000);
                mot2.writeMicroseconds(esc);
                mot3.writeMicroseconds(1000);
                mot4.writeMicroseconds(1000);
            }
            else if (data == '3')
            {
                mot1.writeMicroseconds(1000);
                mot2.writeMicroseconds(1000);
                mot3.writeMicroseconds(esc);
                mot4.writeMicroseconds(1000);
            }
            else if (data == '4')
            {
                mot1.writeMicroseconds(1000);
                mot2.writeMicroseconds(1000);
                mot3.writeMicroseconds(1000);
                mot4.writeMicroseconds(esc);
            }
            else if (data == '5')
            {
                mot1.writeMicroseconds(esc);
                mot2.writeMicroseconds(esc);
                mot3.writeMicroseconds(esc);
                mot4.writeMicroseconds(esc);
            }
        }
        else
        {
            // Wait up to 10 s for throttle to be lowered
            wait_timer = millis() + 10000;

            if (throttle_us > 1050)
            {
                Serial.println(F("Throttle is not in the lowest position."));
                Serial.print(F("Throttle value is: "));
                Serial.println(throttle_us);
                Serial.print(F("Waiting 10 seconds:"));
            }

            while (millis() < wait_timer && !throttle_init_ok)
            {
                getReceiverValues(ch);
                throttle_us = ch[2];
                if (throttle_us < 1050)
                    throttle_init_ok = 1;
                delay(500);
                Serial.print(F("."));
            }
            Serial.println();
        }

        if (!throttle_init_ok)
            data = 'q'; // abort if throttle never came down

        while (loop_timer > micros())
            ;
    }

    // Safe motor shutdown
    mot1.writeMicroseconds(1000);
    mot2.writeMicroseconds(1000);
    mot3.writeMicroseconds(1000);
    mot4.writeMicroseconds(1000);
    loop_counter = 0;

    Serial.println(F("\nVibration check done."));
}