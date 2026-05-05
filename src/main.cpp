///////////////////////////////////////////////////////////////////////////////////////
// Terms of use
///////////////////////////////////////////////////////////////////////////////////////
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
///////////////////////////////////////////////////////////////////////////////////////
// Safety note
///////////////////////////////////////////////////////////////////////////////////////
// Always remove the propellers and stay away from the motors unless you
// are 100% certain of what you are doing.
///////////////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <Wire.h>
#include "receiver/receiver.h" // provides rx_config(), getReceiverValues(), reading_receiver_signals()
#include "test_motor/test_motor.h"
#include "calib_mpu/calib_mpu.h"

// ---------------------------------------------------------------------------
// Manual calibration values  (set use_manual_calibration = true to use them)
// ---------------------------------------------------------------------------
int16_t manual_acc_pitch_cal_value = 196;
int16_t manual_acc_roll_cal_value = 3;

uint8_t use_manual_calibration = false;
int16_t manual_gyro_pitch_cal_value = -14;
int16_t manual_gyro_roll_cal_value = 61;
int16_t manual_gyro_yaw_cal_value = 9;

// Pin definitions are in include/common.h as #define

// ---------------------------------------------------------------------------
// Global variables (shared across modules via common.h / extern declarations)
// ---------------------------------------------------------------------------
uint8_t disable_throttle, flip32;
uint32_t loop_timer;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
int16_t loop_counter;
uint8_t data, start, warning;
int16_t acc_axis[4], gyro_axis[4], temperature;
int32_t gyro_axis_cal[4], acc_axis_cal[4];
int32_t cal_int;
int32_t channel_1, channel_2, channel_3;
int32_t channel_4, channel_5, channel_6;

uint8_t gyro_address = 0x68; // MPU-6050 default I2C address

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------
void print_intro(void);
void red_led(int8_t level);
void green_led(int8_t level);

void print_intro(void)
{
  Serial.println(F(""));
  Serial.println(F("=====         UAV VVD HUS-VNU Setup         ====="));
  Serial.println(F("a = Receiver signals "));
  Serial.println(F("b = I2C scan  "));
  Serial.println(F("c = Gyro raw  "));
  Serial.println(F("d = Accel raw "));
  Serial.println(F("e = IMU angles"));
  Serial.println(F("h = Manual gyro and accelerometer calibration"));
  Serial.println(F("==============================================="));
  Serial.println(F("1 = Check motor 1 (front right, CCW)"));
  Serial.println(F("2 = Check motor 2 (rear right, CW)"));
  Serial.println(F("3 = Check motor 3 (rear left, CCW)"));
  Serial.println(F("4 = Check motor 4 (front left, CW)"));
  Serial.println(F("5 = Check all motors"));
  Serial.println(F("q = Quit current test"));
  Serial.println(F(""));
}

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
void setup()
{
  Serial.begin(57600);

  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);

  // FIX: initialise receiver interrupts — was missing entirely
  rx_config();

  // I2C for MPU-6050: SDA=GPIO11, SCL=GPIO10, 400 kHz
  Wire.begin(11, 10, 400000);

  // Wake the MPU-6050 (PWR_MGMT_1 = 0x00)
  Wire.beginTransmission(gyro_address);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Gyro full-scale: 500 dps  (GYRO_CONFIG = 0x08 -> FS_SEL = 1 -> 65.5 LSB/dps)
  Wire.beginTransmission(gyro_address);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  // Accel full-scale: ±8 g  (ACCEL_CONFIG = 0x10 -> AFS_SEL = 2 -> 4096 LSB/g)
  Wire.beginTransmission(gyro_address);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  // DLPF: ~44 Hz bandwidth  (CONFIG = 0x03)
  Wire.beginTransmission(gyro_address);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();

  // Attach ESCs and arm at 1000 us
  motors_init();

  print_intro();
}

// ---------------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------------
void loop()
{
  delay(10);

  // Update channel globals from ISR buffer every loop iteration
  // FIX: channel_1..6 were never read from the ISR — motors ran on stale/0 values
  {
    uint16_t ch[6];
    getReceiverValues(ch);
    channel_1 = ch[0];
    channel_2 = ch[1];
    channel_3 = ch[2];
    channel_4 = ch[3];
    channel_5 = ch[4];
    channel_6 = ch[5];
  }

  if (Serial.available() > 0)
  {
    data = Serial.read();
    delay(100);
    while (Serial.available() > 0)
      loop_counter = Serial.read();
    disable_throttle = 1;
  }

  if (!disable_throttle)
  {
    mot1.writeMicroseconds(constrain(channel_3, 1000, 2000));
    mot2.writeMicroseconds(constrain(channel_3, 1000, 2000));
    mot3.writeMicroseconds(constrain(channel_3, 1000, 2000));
    mot4.writeMicroseconds(constrain(channel_3, 1000, 2000));
  }
  else
  {
    mot1.writeMicroseconds(1000);
    mot2.writeMicroseconds(1000);
    mot3.writeMicroseconds(1000);
    mot4.writeMicroseconds(1000);
  }

  if (data == 'a')
  {
    Serial.println(F("Reading receiver input pulses."));
    Serial.println(F("Send q to quit."));
    delay(2500);
    reading_receiver_signals();
  }

  if (data == 'b')
  {
    Serial.println(F("Starting the I2C scanner."));
    i2c_scanner();
    data = 0; // reset to avoid running other tests accidentally
    print_intro();
  }

  if (data == 'c')
  {
    Serial.println(F("Reading raw gyro data. Send q to quit."));
    read_gyro_values();
    print_intro();
  }

  if (data == 'd')
  {
    Serial.println(F("Reading raw accelerometer data. Send q to quit."));
    delay(2500);
    read_gyro_values();
    print_intro();
  }

  if (data == 'e')
  {
    Serial.println(F("Reading IMU angles. Send q to quit."));
    check_imu_angles();
    print_intro();
  }

  if (data == 'h')
  {
    Serial.println(F("Manual gyro and accelerometer calibration."));
    manual_imu_calibration();
    data = 0; // reset to avoid running other tests accidentally
    print_intro();
  }

  if (data == '1')
  {
    Serial.println(F("Check motor 1 (front right, CCW). Send q to quit."));
    delay(2500);
    check_motor_vibrations();
    print_intro();
  }

  if (data == '2')
  {
    Serial.println(F("Check motor 2 (rear right, CW). Send q to quit."));
    delay(2500);
    check_motor_vibrations();
    print_intro();
  }

  if (data == '3')
  {
    Serial.println(F("Check motor 3 (rear left, CCW). Send q to quit."));
    delay(2500);
    check_motor_vibrations();
    print_intro();
  }

  if (data == '4')
  {
    Serial.println(F("Check motor 4 (front left, CW). Send q to quit."));
    delay(2500);
    check_motor_vibrations();
    print_intro();
  }

  if (data == '5')
  {
    Serial.println(F("Check all motors. Send q to quit."));
    delay(2500);
    check_motor_vibrations();
    print_intro();
  }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

void red_led(int8_t level)
{
  digitalWrite(RED_LED_PIN, level ? HIGH : LOW);
}

void green_led(int8_t level)
{
  digitalWrite(GREEN_LED_PIN, level ? HIGH : LOW);
}