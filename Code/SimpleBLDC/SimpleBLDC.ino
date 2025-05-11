/*

This is simple Arduino based 650W BLDC motor(with Hall sensors) controller. 
https://github.com/techn0man1ac/SimpleBLDC
Code under develop, not usefull now.

*/

#include <avr/power.h>

#define F_CPU 800000UL  // CPU frequency set to 8 MHz mode

#define PWM_U 10
#define PWM_V 9
#define PWM_W 3
#define IN_U 8
#define IN_V 7
#define IN_W 6
#define HAL_U 13
#define HAL_V 12
#define HAL_W 11
#define Vbatt A0
#define Cur_U A1
#define Cur_V A2
#define Cur_W A3
#define POT A6
#define CWR 4

unsigned long previousMillis = 0;        // will store last time LED was updated
const long interval = 250;           // interval at which to blink (milliseconds)

int time_ = 50;

void setup() {
  clock_prescale_set(clock_div_2);

  Serial.begin(115200);

  pinMode(PWM_U, OUTPUT);
  pinMode(PWM_V, OUTPUT);
  pinMode(PWM_W, OUTPUT);
  pinMode(IN_U, OUTPUT);
  pinMode(IN_V, OUTPUT);
  pinMode(IN_W, OUTPUT);
  pinMode(HAL_U, INPUT);
  pinMode(HAL_V, INPUT);
  pinMode(HAL_W, INPUT);

  setPWMPrescaler(PWM_U, 1);
  setPWMPrescaler(PWM_V, 1);
  setPWMPrescaler(PWM_W, 1);

  analogWrite(PWM_U, 150);
  analogWrite(PWM_V, 150);
  analogWrite(PWM_W, 150);
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    time_ = time_-1;
    time_ = constrain(time_, 12, 50);
  }

  digitalWrite(IN_U, HIGH);
  digitalWrite(IN_V, LOW);
  digitalWrite(IN_W, LOW);
  delay(time_);
  digitalWrite(IN_U, HIGH);
  digitalWrite(IN_V, HIGH);
  digitalWrite(IN_W, LOW);
  delay(time_);
  digitalWrite(IN_U, LOW);
  digitalWrite(IN_V, HIGH);
  digitalWrite(IN_W, LOW);
  delay(time_);
  digitalWrite(IN_U, LOW);
  digitalWrite(IN_V, HIGH);
  digitalWrite(IN_W, HIGH);
  delay(time_);
  digitalWrite(IN_U, LOW);
  digitalWrite(IN_V, LOW);
  digitalWrite(IN_W, HIGH);
  delay(time_);
}

//Atmega 328p (Arduino Uno, Nano)
// Frequencies
// 5, 6   Timer0  62500 Hz
// 9, 10   Timer1  31250 Hz
// 3, 11   Timer2  31250 Hz

// Prescalers
// 5, 6   Timer0  1 8 64 256 1024
// 9, 10   Timer1  1 8 64 256 1024
// 3, 11   Timer2  1 8 32 64 128 256 1024

// Default values
// 5, 6   Timer0 64  977Hz
// 9, 10   Timer1 64  490Hz
// 3, 11   Timer2 64  490Hz

// Consequences
// 5, 6   Timer0  delay() and millis()
// 9, 10   Timer1  Servo library
// 3, 11   Timer2

void setPWMPrescaler(uint8_t pin, uint16_t prescale) {

  byte mode;

  if (pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch (prescale) {
      case    1: mode = 0b001; break;
      case    8: mode = 0b010; break;
      case   64: mode = 0b011; break;
      case  256: mode = 0b100; break;
      case 1024: mode = 0b101; break;
      default: return;
    }

  } else if (pin == 3 || pin == 11) {
    switch (prescale) {
      case    1: mode = 0b001; break;
      case    8: mode = 0b010; break;
      case   32: mode = 0b011; break;
      case   64: mode = 0b100; break;
      case  128: mode = 0b101; break;
      case  256: mode = 0b110; break;
      case 1024: mode = 0b111; break;
      default: return;
    }
  }

  if (pin == 5 || pin == 6) {
    TCCR0B = TCCR0B & 0b11111000 | mode;
  } else if (pin == 9 || pin == 10) {
    TCCR1B = TCCR1B & 0b11111000 | mode;
  } else if (pin == 3 || pin == 11) {
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}
