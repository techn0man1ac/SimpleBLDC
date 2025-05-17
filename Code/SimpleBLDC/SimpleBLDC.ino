/*

    This is simple Arduino based 650W BLDC motor(with Hall sensors) controller.
    https://github.com/techn0man1ac/SimpleBLDC

*/

#include <avr/power.h>

#define F_CPU 800000UL  // CPU frequency set to 8 MHz mode

#define PWM_U 10
#define PWM_V 9
#define PWM_W 3
#define IN_U 8
#define IN_V 7
#define IN_W 6
#define HAL_U 5
#define HAL_V 12
#define HAL_W 11
#define Vbatt A0
#define Cur_U A1
#define Cur_V A2
#define Cur_W A3
#define POT A6
#define CWR 4

unsigned long previousMillis = 0;
#define  interval 250 // Telemetry interval

float BattVoltage = 0.0;

int PWM = 160; // Current limit

void setup() {
  clock_prescale_set(clock_div_2);

  Serial.begin(115200); // Mean 57600(becouse CPU frequency down 2)

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

  MotorRun(0); // Inintalization

  delay(1000);
}

void loop() {
  int Hall_State = HallSensorsRead();
  MotorRun(Hall_State);

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    int sensorValue = analogRead(Vbatt);
    BattVoltage = sensorValue * (55.0 / 1023.0);
    Serial.println(String(Hall_State) + "," + String(BattVoltage));
  }
}

int HallSensorsRead() {
  return digitalRead(HAL_U) + (digitalRead(HAL_V) << 1) + (digitalRead(HAL_W) << 2);
}

void MotorRun(int SensorState) {
  switch (SensorState) {
    case 1: // 100 Step 1
      digitalWrite(IN_U, LOW);
      digitalWrite(IN_V, HIGH);
      digitalWrite(IN_W, LOW);

      analogWrite(PWM_U, PWM);
      analogWrite(PWM_V, PWM);
      analogWrite(PWM_W, 0);
      break;
    case 2: // 010 Step 3
      digitalWrite(IN_U, LOW);
      digitalWrite(IN_V, LOW);
      digitalWrite(IN_W, HIGH);

      analogWrite(PWM_U, 0);
      analogWrite(PWM_V, PWM);
      analogWrite(PWM_W, PWM);
      break;
    case 3: // 110 Step 2
      digitalWrite(IN_U, LOW);
      digitalWrite(IN_V, LOW);
      digitalWrite(IN_W, HIGH);

      analogWrite(PWM_U, PWM);
      analogWrite(PWM_V, 0);
      analogWrite(PWM_W, PWM);
      break;
    case 4: // 001 Step 5
      digitalWrite(IN_U, HIGH);
      digitalWrite(IN_V, LOW);
      digitalWrite(IN_W, LOW);

      analogWrite(PWM_U, PWM);
      analogWrite(PWM_V, 0);
      analogWrite(PWM_W, PWM);
      break;
    case 5: // 101 Step 6
      digitalWrite(IN_U, LOW);
      digitalWrite(IN_V, HIGH);
      digitalWrite(IN_W, LOW);

      analogWrite(PWM_U, 0);
      analogWrite(PWM_V, PWM);
      analogWrite(PWM_W, PWM);
      break;
    case 6: // 011 Step 4
      digitalWrite(IN_U, HIGH);
      digitalWrite(IN_V, LOW);
      digitalWrite(IN_W, LOW);

      analogWrite(PWM_U, PWM);
      analogWrite(PWM_V, PWM);
      analogWrite(PWM_W, 0);
      break;

    default:
      // turn all the off:
      digitalWrite(IN_U, LOW);
      digitalWrite(IN_V, LOW);
      digitalWrite(IN_W, LOW);
      analogWrite(PWM_U, 0);
      analogWrite(PWM_V, 0);
      analogWrite(PWM_W, 0);
  }
}

void setPWMPrescaler(uint8_t pin, uint16_t prescale) { // How to change the PWM frequency on Arduino https://www.luisllamas.es/en/change-pwm-frequency-arduino/

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
