#include "Arduino.h"
#include <EEPROM.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DS3231.h>

#define row2pix(size, pos) ((size) * (pos) * 7)

#define rotary_encoder_pin_a 3
#define rotary_encoder_pin_b 4
#define rotary_encoder_button 5

#define door_up_button 8
#define door_down_button 9

#define door_max_running_time 5000
#define door_on 6
#define door_in3 2
#define door_in4 7

#define enter_settings_hold_time 2000
#define blink_time_clock 500

#define time_til_idle 60000
unsigned long previous_activity = 0;
bool idle = true;
bool wake_up = false;

enum door_state {
        DOOR_UP,
        DOOR_DOWN,
        DOOR_MIDDLE,
};

Adafruit_SSD1306 display(128, 64, &Wire, 4);

#define LOGO_WIDTH 128
#define LOGO_HEIGHT 64
const unsigned char chicken_logo[1024] PROGMEM = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x1f, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x80, 0x00, 0x00,
        0x00, 0x00, 0x00, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xc0, 0x00, 0x00,
        0x00, 0x00, 0x01, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xe0, 0x00, 0x00,
        0x00, 0x00, 0x01, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xe0, 0x00, 0x00,
        0x00, 0x00, 0x01, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xe0, 0x00, 0x00,
        0x00, 0x00, 0x03, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xf0, 0x00, 0x00,
        0x00, 0x00, 0x07, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xf8, 0x00, 0x00,
        0x00, 0x00, 0x03, 0xff, 0xff, 0x80, 0x00, 0x00, 0x7f, 0xe3, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x00,
        0x00, 0x00, 0x01, 0xff, 0xff, 0xf0, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00,
        0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf4, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xfb, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xf1, 0xe1, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x1e, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x0e, 0x70, 0x00, 0x00, 0x0f, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x07, 0xf0, 0x00, 0x00, 0x07, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x03, 0xe0, 0x00, 0x00, 0x07, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x0f, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x19, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x80, 0x00, 0x10, 0xc0, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xc0, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

DS3231 rt_clock;
int hour = 23;
bool century = false;
bool h12Flag;
bool pmFlag;

bool auto_door_up = false;
bool auto_door_down = false;

//                    winter  |  summer
uint8_t open_hour[2]  {11     ,    11};
uint8_t close_hour[2] {23     ,    19};
uint8_t is_winter = false;
#define chs_adr 100
#define chw_adr 101
#define ohs_adr 102
#define ohw_adr 103
#define isw_adr 104

volatile unsigned long previous_encoder_step = 0;
volatile unsigned int encoder_step = 0;
unsigned int max_step_size = -1;
#define encoder_timeout 50

void do_rotary_encoder_step()
{
        unsigned long t = millis();
        if (t - previous_encoder_step <= encoder_timeout) return;
        previous_encoder_step = t;
        wake_up = true;
        delay(1);

        if (digitalRead(rotary_encoder_pin_b) == LOW) {
                if (encoder_step == 0)
                        encoder_step = max_step_size;
                else
                        encoder_step--;
        } else {
                encoder_step++;
        }

        if (encoder_step > max_step_size)
                encoder_step = 0;
}

void wake_screen()
{
        if (!idle) return;

        display.clearDisplay();
        display.drawBitmap(0, 0, chicken_logo, LOGO_WIDTH, LOGO_HEIGHT, 1);
        display.display();
        delay(200);
        idle = false;
        previous_activity = millis();
        return;
}

void setup()
{
        // set pins
        attachInterrupt(digitalPinToInterrupt(rotary_encoder_pin_a), do_rotary_encoder_step, CHANGE);
        pinMode(rotary_encoder_pin_b,  INPUT);
        pinMode(rotary_encoder_button, INPUT);
        pinMode(door_up_button,        INPUT);
        pinMode(door_down_button,      INPUT);
        pinMode(door_on,               OUTPUT);
        pinMode(door_in3,              OUTPUT);
        pinMode(door_in4,              OUTPUT);
        digitalWrite(door_in3,         LOW);
        digitalWrite(door_in4,         LOW);
        digitalWrite(door_on,          LOW);

        // Read EEPROM
        if (EEPROM.read(chs_adr) > 23) EEPROM.write(chs_adr, close_hour[0]);
        if (EEPROM.read(chw_adr) > 23) EEPROM.write(chw_adr, close_hour[0]);
        if (EEPROM.read(ohs_adr) > 23) EEPROM.write(ohs_adr, open_hour[1]);
        if (EEPROM.read(ohw_adr) > 23) EEPROM.write(ohw_adr, open_hour[0]);
        if (EEPROM.read(isw_adr) > 1)  EEPROM.write(isw_adr, is_winter);
        open_hour[1]                 = EEPROM.read(ohs_adr);
        open_hour[0]                 = EEPROM.read(ohw_adr);
        close_hour[1]                = EEPROM.read(chs_adr);
        close_hour[0]                = EEPROM.read(chw_adr);
        is_winter                    = EEPROM.read(isw_adr);

        // start I2C
        Wire.begin();

        if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
                // Serial.println(F("SSD1306 allocation failed"));
                for(;;);
        }
        display.setTextSize(2);
        display.setTextColor(SSD1306_WHITE);
        wake_screen();

        delay(500);
}

enum door_state check_door()
{
        uint8_t up   = digitalRead(door_up_button);
        uint8_t down = digitalRead(door_down_button);

        if (down == up) return DOOR_MIDDLE;
        if (up == HIGH)  return DOOR_UP;
        return DOOR_DOWN;
}

void open_door()
{
        digitalWrite(door_in3, HIGH);
        digitalWrite(door_in4, LOW);
        digitalWrite(door_on, HIGH);

        unsigned long start_time = millis();
        while(digitalRead(door_up_button) == LOW && millis() - start_time < door_max_running_time);

        delay(10);
        digitalWrite(door_in3, LOW);
        digitalWrite(door_in4, LOW);
        digitalWrite(door_on, LOW);
}

void close_door()
{
        digitalWrite(door_in3, LOW);
        digitalWrite(door_in4, HIGH);
        digitalWrite(door_on, HIGH);

        unsigned long start_time = millis();
        while(digitalRead(door_down_button) == LOW && millis() - start_time < door_max_running_time);

        delay(10);
        digitalWrite(door_in3, LOW);
        digitalWrite(door_in4, LOW);
        digitalWrite(door_on, LOW);
}

void check_time_draw()
{
        int prev_hour = hour;
        hour = rt_clock.getHour(h12Flag, pmFlag);
        enum door_state state = check_door();

        // new day
        if (hour < prev_hour) {
                auto_door_down = false;
                auto_door_up = false;
        }

        if (hour >= close_hour[is_winter] && !auto_door_down) {
                auto_door_up = true;
                auto_door_down = true;
                if (state == DOOR_UP) close_door();
        } else if (hour >= open_hour[is_winter] && !auto_door_up) {
                auto_door_up = true;
                if (state == DOOR_DOWN) open_door();
        }

        display.clearDisplay();
        display.setCursor(0, 0);

        if (state == DOOR_UP) {
                display.print("Luka er no\noppe");
        } else if (state == DOOR_DOWN) {
                display.print("Luka er no\nnede");
        } else {
                display.print("Error:\nkan ikkje\nfinne luka");
                goto show_display;
        }
        display.print(":\nVeksle med\neit trykk");

        show_display:
        display.display();
}

inline void toggle_door()
{
        enum door_state state = check_door();
        if (state == DOOR_UP) close_door();
        else                  open_door();

}

void set_time()
{
        bool unpressed = false;
        max_step_size = 23;
        encoder_step = hour;

        uint8_t minute = rt_clock.getMinute();
        bool min = false;

        bool clock_off = false;
        unsigned long last_change = millis();

        for(;;) {
                display.setCursor(37, 22);

                if (min) {
                        if (hour < 10) display.print("0");
                        display.print(hour);
                        display.print(":");
                }

                // blink hour or min
                if (!clock_off) {
                        if (encoder_step < 10) display.print("0");
                        display.print(encoder_step);

                        unsigned long t = millis();
                        if (t - last_change >= blink_time_clock) {
                                clock_off = true;
                                last_change = t;
                        }
                } else {
                        display.print("  ");

                        unsigned long t = millis();
                        if (t - last_change >= blink_time_clock / 3) {
                                clock_off = false;
                                last_change = t;
                        }
                }

                if (!min) {
                        display.print(":");
                        if (minute < 10) display.print("0");
                        display.print(minute);
                }

                display.display();
                display.clearDisplay();

                if (!unpressed) {
                        if (digitalRead(rotary_encoder_button) == HIGH) {
                                delay(50);
                                unpressed = true;
                        }
                } else if (digitalRead(rotary_encoder_button) == LOW) {
                        if (!min) {
                                hour = encoder_step;
                                unpressed = false;
                                max_step_size = 60;
                                encoder_step = minute;
                                min = true;
                        } else {
                                minute = encoder_step;
                                break;
                        }
                }
        }

        rt_clock.setHour(hour);
        rt_clock.setMinute(minute);
}

void toggle_winter_summer_time()
{
        is_winter = !is_winter;
        const char* mode[2] = {"Vinter", "Sommer"};

        display.setCursor(0, 0);
        display.print("Sesong\nstilt:");

        display.setCursor(28, 42);
        display.print(mode[is_winter]);
        display.display();

        if (EEPROM.read(isw_adr) != is_winter) EEPROM.write(isw_adr, is_winter);
        delay(1500);
}

void up_down_time()
{
        max_step_size = 23;
        encoder_step = open_hour[is_winter];

        bool unpressed = false;
        bool up = true;
        bool clock_off = false;
        unsigned long last_change = 0;

        for(;;) {
                display.setCursor(0, 0);
                if (up) display.print("Oppe: ");
                else    display.print("Lukk: ");

                // blink clock
                unsigned long t = millis();
                if (!clock_off) {
                        if (encoder_step < 10) display.print("0");
                        display.print(encoder_step);

                        if (t - last_change >= blink_time_clock) {
                                clock_off = true;
                                last_change = t;
                        }
                } else if (t - last_change >= blink_time_clock / 3) {
                        clock_off = false;
                        last_change = t;
                }

                display.display();
                display.clearDisplay();

                if (!unpressed) {
                        if (digitalRead(rotary_encoder_button) == HIGH) {
                                delay(50);
                                unpressed = true;
                        }
                } else if (digitalRead(rotary_encoder_button) == LOW) {
                        if (up) {
                                up = false;
                                open_hour[is_winter] = encoder_step;
                                encoder_step = close_hour[is_winter];
                                unpressed = false;
                        } else {
                                close_hour[is_winter] = encoder_step;
                                break;
                        }
                }
        }
        if (EEPROM.read(chs_adr) != close_hour[1]) EEPROM.write(chs_adr, close_hour[1]);
        if (EEPROM.read(chw_adr) != close_hour[0]) EEPROM.write(chw_adr, close_hour[0]);
        if (EEPROM.read(ohs_adr) != open_hour[1])  EEPROM.write(ohs_adr, open_hour[1]);
        if (EEPROM.read(ohw_adr) != open_hour[0])  EEPROM.write(ohw_adr, open_hour[0]);
}

void settings()
{
        bool unpressed = false;
        max_step_size = 2;
        encoder_step = 0;
        const uint8_t padding = 5;

        const char* setting_names[3] = {"Endre tid", "Sesong", "Luke tid"};
        void (*setting_funcs[3])(void) = {set_time, toggle_winter_summer_time, up_down_time};

        for(;;) {
                for (uint8_t i = 0; i < 3; i++) {
                        display.setCursor(0, row2pix(2, i) + padding * i);
                        if (i == encoder_step) display.print(">");
                        display.print(setting_names[i]);
                }

                display.display();
                display.clearDisplay();

                if (!unpressed) {
                        if (digitalRead(rotary_encoder_button) == HIGH) {
                                delay(50);
                                unpressed = true;
                        }
                } else if (digitalRead(rotary_encoder_button) == LOW) {
                        unsigned long start_time = millis();
                        delay(20);

                        // exit settings
                        while(digitalRead(rotary_encoder_button) == LOW)  {
                                if (millis() - start_time >= enter_settings_hold_time) {
                                        idle = true;
                                        return;
                                }
                        }
                        // enter setting
                        setting_funcs[encoder_step]();
                        return;
                }

                if (!idle && millis() - previous_activity >= time_til_idle) {
                        return;
                }
        }
}

void loop()
{
        if (wake_up) {
                wake_screen();
                wake_up = false;
        }

        if (!idle) {
                check_time_draw();
                if (millis() - previous_activity >= time_til_idle) {
                        display.clearDisplay();
                        display.display();
                        idle = true;
                }
        }
        if (digitalRead(rotary_encoder_button) == LOW) {
                unsigned long start_time = millis();
                wake_screen();
                delay(20);

                // enter settings
                while(digitalRead(rotary_encoder_button) == LOW)  {
                        if (millis() - start_time >= enter_settings_hold_time) {
                                settings();

                                idle = true;
                                wake_screen();
                                while(digitalRead(rotary_encoder_button) == LOW);
                                delay(700);
                                return;
                        }
                }
                toggle_door();
        }
}
