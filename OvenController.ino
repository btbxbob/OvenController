#include <LCD4884.h>
#include <MAX6675.h>

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
#define JOYSTICK_DIRECTION_SMALL 200
#define JOYSTICK_DIRECTION_BIG 400

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

char get_key_joystick() {
  int x, y, z;
  x = analogRead(JoyStick_X);
  y = analogRead(JoyStick_Y);
  z = digitalRead(JoyStick_Z);
  if (z == 0) return CENTER_KEY;
  if (x < JOYSTICK_MID - JOYSTICK_DIRECTION_BIG) return LEFT_KEY;
  if (y < JOYSTICK_MID - JOYSTICK_DIRECTION_BIG) return DOWN_KEY;
  if (x > JOYSTICK_MID + JOYSTICK_DIRECTION_BIG) return RIGHT_KEY;
  if (y > JOYSTICK_MID + JOYSTICK_DIRECTION_BIG) return UP_KEY;
  return -1;
}

void waitfor_OKkey() {
  byte i;
  byte key = 0xFF;
  while (key != CENTER_KEY) {
    for (i = 0; i < NUM_KEYS; i++) {
      if (button_flag[i] != 0) {
        button_flag[i] = 0;  // reset button flag
        if (i == CENTER_KEY) key = CENTER_KEY;
      }
    }
  }
}

void temperature(float t) {
  char temperature_char[3];
  int len = sprintf(temperature_char, "%3i", (int)t);
  Serial.println(temperature_char);
  Serial.println(t);
  Serial.println(len);
  lcd.LCD_write_string(0, 1, "Curr:", MENU_NORMAL);
  lcd.LCD_write_string(0, 4, "Targ:", MENU_NORMAL);
  lcd.LCD_write_string_big(40, 0, temperature_char, MENU_NORMAL);
  lcd.LCD_write_string_big(40, 3, "300", MENU_NORMAL);
  // lcd.LCD_write_string(33, 5, "MENU", MENU_HIGHLIGHT );
}

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

void setup() {
  // joystick
  pinMode(JoyStick_Z, INPUT);
  // Serial
  Serial.begin(9600);
  // setup interrupt-driven keypad arrays
  // reset button arrays
  for (byte i = 0; i < NUM_KEYS; i++) {
    button_count[i] = 0;
    button_status[i] = 0;
    button_flag[i] = 0;
  }

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

  SREG |= 1 << SREG_I;

  lcd.LCD_init();
  lcd.LCD_clear();

  lcd.backlight(ON);  // Turn on the backlight
  // lcd.backlight(OFF); // Turn off the backlight
}

void loop() {
  // Serial.print(ts.read_temp(), 2);
  // Serial.print(" C \n");

  temperature(ts.read_temp());
  delay(2000);
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

// Timer2 interrupt routine -
// 1/(160000000/256/(256-6)) = 4ms interval

ISR(TIMER2_OVF_vect) {
  TCNT2 = 6;
  update_adc_key();
}
