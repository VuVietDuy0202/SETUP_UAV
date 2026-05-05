#include "common.h"

#define NUM_CHANNELS 6

// ======================= PIN CONFIG =======================
const uint8_t channel_pins[NUM_CHANNELS] = {37, 38, 39, 40, 41, 42};

// ======================= DATA =======================
volatile uint32_t timer_ch[NUM_CHANNELS];
volatile uint16_t ReceiverValue[NUM_CHANNELS] = {1500, 1500, 1000, 1500, 1000, 1000};

// ======================= ISR CORE =======================
void IRAM_ATTR handle_channel(uint8_t ch)
{
    uint32_t t = micros();
    uint8_t pin = channel_pins[ch];
    uint8_t state;

    if (pin < 32)
        state = (GPIO.in >> pin) & 0x1;
    else
        state = (GPIO.in1.val >> (pin - 32)) & 0x1;

    if (state)
    {
        timer_ch[ch] = t;
    }
    else
    {
        uint32_t width = t - timer_ch[ch];
        if (width > 900 && width < 2100)
            ReceiverValue[ch] = (uint16_t)width;
    }
}

// ======================= ISR WRAPPERS =======================
void IRAM_ATTR ch1_ISR() { handle_channel(0); }
void IRAM_ATTR ch2_ISR() { handle_channel(1); }
void IRAM_ATTR ch3_ISR() { handle_channel(2); }
void IRAM_ATTR ch4_ISR() { handle_channel(3); }
void IRAM_ATTR ch5_ISR() { handle_channel(4); }
void IRAM_ATTR ch6_ISR() { handle_channel(5); }

// ======================= INIT =======================
void rx_config(void)
{
    for (int i = 0; i < NUM_CHANNELS; i++)
        pinMode(channel_pins[i], INPUT);

    attachInterrupt(channel_pins[0], ch1_ISR, CHANGE);
    attachInterrupt(channel_pins[1], ch2_ISR, CHANGE);
    attachInterrupt(channel_pins[2], ch3_ISR, CHANGE);
    attachInterrupt(channel_pins[3], ch4_ISR, CHANGE);
    attachInterrupt(channel_pins[4], ch5_ISR, CHANGE);
    attachInterrupt(channel_pins[5], ch6_ISR, CHANGE);
}

// ======================= SAFE READ =======================
void getReceiverValues(uint16_t *ch)
{
    noInterrupts();
    for (int i = 0; i < NUM_CHANNELS; i++)
        ch[i] = ReceiverValue[i];
    interrupts();
}

// ======================= RECEIVER DISPLAY LOOP =======================
void reading_receiver_signals(void)
{
    uint16_t ch[NUM_CHANNELS];

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

        // FIX: always fetch fresh receiver values from ISR buffer
        getReceiverValues(ch);
        channel_1 = ch[0];
        channel_2 = ch[1];
        channel_3 = ch[2];
        channel_4 = ch[3];
        channel_5 = ch[4];
        channel_6 = ch[5];

        // Motor arm/disarm logic (for display purposes only in this test)
        if (channel_3 < 1100 && channel_4 < 1300)
            start = 1;
        if (start == 1 && channel_3 < 1100 && channel_4 > 1450)
            start = 2;
        if (start == 2 && channel_3 < 1300 && channel_4 > 1700)
            start = 0;

        Serial.print("Start:");
        Serial.print(start);

        Serial.print("  Roll:");
        if (channel_1 - 1480 < 0)
            Serial.print("<<<");
        else if (channel_1 - 1520 > 0)
            Serial.print(">>>");
        else
            Serial.print("-+-");
        Serial.print(channel_1);

        Serial.print("  Pitch:");
        if (channel_2 - 1480 < 0)
            Serial.print("^^^");
        else if (channel_2 - 1520 > 0)
            Serial.print("vvv");
        else
            Serial.print("-+-");
        Serial.print(channel_2);

        Serial.print("  Throttle:");
        if (channel_3 - 1480 < 0)
            Serial.print("vvv");
        else if (channel_3 - 1520 > 0)
            Serial.print("^^^");
        else
            Serial.print("-+-");
        Serial.print(channel_3);

        Serial.print("  Yaw:");
        if (channel_4 - 1480 < 0)
            Serial.print("<<<");
        else if (channel_4 - 1520 > 0)
            Serial.print(">>>");
        else
            Serial.print("-+-");
        Serial.print(channel_4);

        Serial.print("  CH5:");
        Serial.print(channel_5);
        Serial.print("  CH6:");
        Serial.println(channel_6);
    }

    print_intro();
}