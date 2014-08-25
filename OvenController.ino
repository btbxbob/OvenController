#include <LCD4884.h>
//84x48
#include <MAX6675.h>
#include "Event.h"
#include "Timer.h"
#include "Timer4.h"
#include "EEPROMex.h"
#include "Timer3.h"
#include "idle.h"
#include "heating.h"
#include "cooling.h"
#include "bulb.h"

// keypad debounce parameter
#define DEBOUNCE_MAX 15
#define DEBOUNCE_ON 10
#define DEBOUNCE_OFF 3

#define NUM_KEYS 5

#define NUM_MENU_ITEM 4

// joystick number
#define LEFT_KEY 0
#define CENTER_KEY 1
#define DOWN_KEY 2
#define RIGHT_KEY 3
#define UP_KEY 4

// PS2 joystick
#define JoyStick_X 2
#define JoyStick_Y 1
#define JoyStick_Z 11
#define JOYSTICK_MID 500
#define JOYSTICK_DIRECTION_SMALL 100
#define JOYSTICK_DIRECTION_BIG 400

// Relay
#define RELAY_PORT 12
#define RELAY_ON HIGH
#define RELAY_OFF LOW

// menu starting points

#define MENU_X 10  // 0-83
#define MENU_Y 1   // 0-5

int adc_key_val[5] = {50, 200, 400, 600, 800};

// debounce counters
byte button_count[NUM_KEYS];
// button status - pressed/released
byte button_status[NUM_KEYS];
// button on flags for user program
byte button_flag[NUM_KEYS];

MAX6675 ts(9, 8, 10, 1);
// MAX6675 temp0(CS,SO,SCK,units);
// Max6675 module: SO on pin #8, SS on pin #9, CSK on pin #10 of Arduino UNO
// Other pins are capable to run this library, as long as digitalRead works on
// SO,
// and digitalWrite works on SS and CSK
/*
units is one of the following:
0 = raw 0-4095 value
1 = temp in ˚C
2 = temp in ˚F
*/

#define TEMPERATURE_BIG 1
#define TEMPERATURE_SMALL 0

// tolerance: turn relay when temperature is smaller than (target - tolerance)
// or bigger than (target + tolerance)
#define TEMPERATURE_TOLERANCE 1

struct temperatureMark {
  bool enable = 0;
  byte x;
  byte y;
  bool size;  // 0: small 1: big
} tm1;

// histories
#define TEMPERATURE_HISTORY_MAX 10
int temperature_history[TEMPERATURE_HISTORY_MAX] = {0};
int temperature_history_count[TEMPERATURE_HISTORY_MAX] = {0};

// oven state
#define OVEN_STATE_INIT 0
#define OVEN_HEATING 1
#define OVEN_COOLING 2
#define OVEN_STABLE 3
#define OVEN_DOOR_OPEN 4
#define OVEN_IDLE 5
#define OVEN_UNKNOW 6
byte ovenState=OVEN_STATE_INIT;

int targetTemperature;
float temperature_old = 0;
bool write_lcd_lock = false;

float get_temperature() {
  float t = ts.read_temp();
  // calibrate here
  return t;
}

void turn_relay(bool value) {
  if (value == RELAY_ON) {
    digitalWrite(RELAY_PORT, RELAY_ON);
  } else if (value == RELAY_OFF) {
    digitalWrite(RELAY_PORT, RELAY_OFF);
  }
}

void write_left_icon(const unsigned char * icon)
{
  lcd.LCD_draw_bmp_pixel(0,0, (unsigned char *)icon, 32, 48);
}

void write_oven_state(byte inOvenState) {
  //char ovenStateChar[5];
  switch (inOvenState)
  {
    case OVEN_IDLE:
      write_left_icon(idle);
    break;
    case OVEN_HEATING:
      write_left_icon(heating);
    break;
    case OVEN_COOLING:
      write_left_icon(cooling);
    break;
    case OVEN_STABLE:
      write_left_icon(bulb);
    break;
  }
}

void write_temperature(int x, int y, bool size) {
  if (write_lcd_lock == false) {  // check lock
    tm1.x = x;
    tm1.y = y;
    tm1.size = size;
    tm1.enable = true;
    // temperature:
    // max: 800
    // min: 0
    float t = get_temperature();
    if (t != temperature_old) {

      char temperature_char[5];
      int len = snprintf(temperature_char, 5, "%3i", (int)t);
      write_lcd_lock = true;  // lock here
      if (size == TEMPERATURE_BIG) {
        lcd.LCD_write_string_big(x, y, temperature_char, MENU_NORMAL);
      } else {
        lcd.LCD_write_string(x, y, temperature_char, MENU_NORMAL);
      }
      write_lcd_lock = false;  // unlock
      temperature_old = t;
    }
  }
}

char get_key_joystick() {
  int x, y, z;
  x = analogRead(JoyStick_X);
  y = analogRead(JoyStick_Y);
  z = digitalRead(JoyStick_Z);
  if (z == 0) return CENTER_KEY;
  if (x < JOYSTICK_MID - JOYSTICK_DIRECTION_SMALL) return LEFT_KEY;
  if (y < JOYSTICK_MID - JOYSTICK_DIRECTION_SMALL) return DOWN_KEY;
  if (x > JOYSTICK_MID + JOYSTICK_DIRECTION_SMALL) return RIGHT_KEY;
  if (y > JOYSTICK_MID + JOYSTICK_DIRECTION_SMALL) return UP_KEY;
  return -1;
}

byte wait_for_key() {
  byte i;
  byte key = 0xFF;
  while (1) {
    for (i = 0; i < NUM_KEYS; i++) {
      if (button_flag[i] != 0) {
        return i;
      }
    }
  }
}

void temperature() {
  //lcd.LCD_write_string(1, 1, "Curr:", MENU_NORMAL);
  //lcd.LCD_write_string(1, 4, "Targ:", MENU_NORMAL);
  write_left_icon(idle);
  write_temperature(33, 0, TEMPERATURE_BIG);
  tm1.enable = true;  // start renew temperature，REMEMBER to turn off.
  int targetTemperature_now = targetTemperature;
  char targetTemperature_char[4];
  snprintf(targetTemperature_char, 4, "%3i", targetTemperature);
  lcd.LCD_write_string_big(33, 3, targetTemperature_char, MENU_NORMAL);
  // lcd.LCD_write_string(33, 5, "MENU", MENU_HIGHLIGHT );
  // wait for keys here.
  while (1) {
    byte key = wait_for_key();
    int x = analogRead(JoyStick_X);
    int y = analogRead(JoyStick_Y);
    // Serial.println(key);
    byte step = 5;
    if (abs(x - 500) + abs(y - 500) > 450) step = 10;
    if (abs(x - 500) + abs(y - 500) < 350) step = 1;
    if (key == UP_KEY) {
      if (targetTemperature_now <= 270 - step) {
        targetTemperature_now += step;
      } else {
        targetTemperature_now = 270;
      }
    } else if (key == DOWN_KEY) {
      if (targetTemperature_now >= step) {
        targetTemperature_now -= step;
      } else {
        targetTemperature_now = 0;
      }
    } else if (key == CENTER_KEY) {
      EEPROM.writeInt(0, targetTemperature_now);
      targetTemperature = targetTemperature_now;
    }
    if (write_lcd_lock == false) {
      snprintf(targetTemperature_char, 4, "%3i", targetTemperature_now);
      write_lcd_lock = true;  // lock
      lcd.LCD_write_string_big(33, 3, targetTemperature_char, MENU_NORMAL);
      write_lcd_lock = false;  // unlock
      delay(100);
    }
  }
}

void setup() {
  // joystick
  pinMode(JoyStick_Z, INPUT);
  // Relay
  pinMode(RELAY_PORT, OUTPUT);
  // Serial
  Serial.begin(9600);
  // setup interrupt-driven keypad arrays
  // reset button arrays
  for (byte i = 0; i < NUM_KEYS; i++) {
    button_count[i] = 0;
    button_status[i] = 0;
    button_flag[i] = 0;
  }

#ifdef __AVR_ATmega32U4__
  startTimer3(10000L);
#else

  // Setup timer2 -- Prescaler/256
  TCCR2A &= ~((1 << WGM21) | (1 << WGM20));
  TCCR2B &= ~(1 << WGM22);
  TCCR2B = (1 << CS22) | (1 << CS21);

  ASSR |= (0 << AS2);

  // Use normal mode
  TCCR2A = 0;
  // Timer2 Overflow Interrupt Enable
  TIMSK2 |= (0 << OCIE2A);
  TCNT2 = 0x6;  // counting starts from 6;
  TIMSK2 = (1 << TOIE2);
#endif

  SREG |= 1 << SREG_I;

  lcd.LCD_init();
  lcd.LCD_clear();

  lcd.backlight(ON);      // Turn on the backlight
                          // lcd.backlight(OFF); // Turn off the backlight
  startTimer4(2000000L);  // 2s

  targetTemperature = EEPROM.readInt(0);
  // digitalWrite(RELAY_PORT, LOW);
}

byte judge_oven_state(){
  //stable
  
}

ISR(timer4Event) {
  // every 2s
  resetTimer4();

  // 3 jobs here:
  // 1. Refresh *every* temperature display
  if (tm1.enable) {
    write_temperature(tm1.x, tm1.y, tm1.size);
  }
  // 2. check and control the oven relay
  float t = get_temperature();
  // record the histoy of temperatures
  if (abs(floor(t) - temperature_history[0])>1) {
    for (int i = 1; i < TEMPERATURE_HISTORY_MAX; i++) {
      temperature_history[i] = temperature_history[i - 1];
      temperature_history_count[i] = temperature_history[i - 1];
    }
    temperature_history[0]=floor(t);
  } else {
    temperature_history_count[0]++;
    //Serial.print("temp count: ");
    //Serial.println(temperature_history_count[0]);
  }
  //Serial.print("temp history 0: ");
  Serial.print(t);
  Serial.print(",");
  ovenState=judge_oven_state();
  write_oven_state(ovenState);
  if (t < targetTemperature - TEMPERATURE_TOLERANCE) {
    turn_relay(RELAY_ON);
    Serial.print("ON");
  } else if (t >= targetTemperature + TEMPERATURE_TOLERANCE) {
    turn_relay(RELAY_OFF);
    Serial.print("OFF");
  }
  Serial.println(";");
  // 3. blink chars
}

void loop() {
  // Serial.print(get_temperature(), 2);
  // Serial.print(" C \n");
  temperature();
  delay(3000);
}

char get_key(unsigned int input) {
  char k;
  for (k = 0; k < NUM_KEYS; k++) {
    if (input < adc_key_val[k]) {
      return k;
    }
  }

  if (k >= NUM_KEYS) k = -1;  // No valid key pressed
  return k;
}

void update_adc_key() {
  int adc_key_in;
  char key_in;
  byte i;

  adc_key_in = analogRead(0);
  key_in = get_key(adc_key_in);
  if (key_in == -1) key_in = get_key_joystick();
  // Serial.println("update_adc_key");
  for (i = 0; i < NUM_KEYS; i++) {
    if (key_in == i)  // one key is pressed
    {
      if (button_count[i] < DEBOUNCE_MAX) {
        button_count[i]++;
        if (button_count[i] > DEBOUNCE_ON) {
          if (button_status[i] == 0) {
            button_flag[i] = 1;
            button_status[i] = 1;  // button debounced to 'pressed' status
          }
        }
      }
    } else  // no button pressed
    {
      if (button_count[i] > 0) {
        button_flag[i] = 0;
        button_count[i]--;
        if (button_count[i] < DEBOUNCE_OFF) {
          button_status[i] = 0;  // button debounced to 'released' status
        }
      }
    }
  }
}

#ifdef __AVR_ATmega32U4__
ISR(timer3Event) {
  resetTimer3();
  update_adc_key();
}
#else
// Timer2 interrupt routine -
// 1/(160000000/256/(256-6)) = 4ms interval

ISR(TIMER2_OVF_vect) {
  TCNT2 = 6;
  update_adc_key();
}
#endif
