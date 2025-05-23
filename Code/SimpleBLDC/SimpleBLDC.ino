/*

    This is simple Arduino based 650W BLDC motor(with Hall sensors) controller.
    https://github.com/techn0man1ac/SimpleBLDC

*/

#include <avr/power.h>

//#define F_CPU 800000UL  // CPU frequency set to 8 MHz mode

#define PWM_U 10
#define PWM_V 9
#define PWM_W 3
#define IN_U 8
#define IN_V 11
#define IN_W 12
#define HAL_U 5
#define HAL_V 6
#define HAL_W 7
#define Vbatt A0
#define Cur_U A1
#define Cur_V A2
#define Cur_W A3
#define POT A6
#define CWR 4

volatile byte Hall_State = 0;

float BattVoltage = 0.0;
int VoltageRAW = 0;

int U_CurrentRAW = 0;
int V_CurrentRAW = 0;
int W_CurrentRAW = 0;

float U_Current = 0.0;
float V_Current = 0.0;
float W_Current = 0.0;

float Voltage_Coef = (55.0 / 1023.0);
float U_Current_Coef = (5.0 / 511.0);
float V_Current_Coef = (5.0 / 511.0);
float W_Current_Coef = (5.0 / 511.0);

int PWM = 165; // Current limit

unsigned long previousMillis = 0;
#define  interval 100 // Telemetry interval

void setup() {
  //clock_prescale_set(clock_div_2); // Set CPU to 8 MHz mode

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
  
  // Using Arduino Interrupts https://dronebotworkshop.com/interrupts/
  // Enable PCIE2 Bit2 = 1 (Port D)
  PCICR |= B00000100;
  // Select PCINT21, PCINT22, PCINT23 = 1 (Pin D5, D6, D7)
  PCMSK2 |= B11100000;

  MotorRun(0); // Inintalization
  delay(1000);
  Hall_State = HallSensorsRead();
  MotorRun(Hall_State);
}

void loop() {
  U_CurrentRAW = analogRead(Cur_U);
  V_CurrentRAW = analogRead(Cur_V);
  W_CurrentRAW = analogRead(Cur_W);

  VoltageRAW = analogRead(Vbatt);

  BattVoltage = VoltageRAW * Voltage_Coef;

  U_Current = (U_CurrentRAW - 518) * U_Current_Coef;
  V_Current = (V_CurrentRAW - 518) * V_Current_Coef;
  W_Current = (W_CurrentRAW - 516) * W_Current_Coef;

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    Serial.println(String(U_Current) + "," + String(V_Current) + "," + String(W_Current) + "," + String(Hall_State) + "," + String(BattVoltage));
  }
}

ISR (PCINT2_vect){
  Hall_State = HallSensorsRead();
  MotorRun(Hall_State);
}

int HallSensorsRead() {
  return PIND >> 5;
}

// Six-Step (Trapezoidal) Commutation https://onlinedocs.microchip.com/oxy/GUID-ED5A4EC0-70D9-40E8-82A4-9EE711C91C4E-en-US-11/GUID-A2E50F02-E898-425F-B2A2-36E161DE810B.html
void MotorRun(int SensorState) {
  switch (SensorState) {
    case 1: // 100 Step 4
      analogWrite(PWM_U, PWM); digitalWrite(IN_U, LOW);
      analogWrite(PWM_V, PWM); digitalWrite(IN_V, HIGH);
      analogWrite(PWM_W, 0);   digitalWrite(IN_W, LOW);
      break;

    case 2: // 010 Step 6
      analogWrite(PWM_U, 0);   digitalWrite(IN_U, LOW);
      analogWrite(PWM_V, PWM); digitalWrite(IN_V, LOW);
      analogWrite(PWM_W, PWM); digitalWrite(IN_W, HIGH);
      break;

    case 3: // 110 Step 5
      analogWrite(PWM_U, PWM); digitalWrite(IN_U, LOW);
      analogWrite(PWM_V, 0);   digitalWrite(IN_V, LOW);
      analogWrite(PWM_W, PWM); digitalWrite(IN_W, HIGH);
      break;

    case 4: // 001 Step 2
      analogWrite(PWM_U, PWM); digitalWrite(IN_U, HIGH);
      analogWrite(PWM_V, 0);   digitalWrite(IN_V, LOW);
      analogWrite(PWM_W, PWM); digitalWrite(IN_W, LOW);
      break;

    case 5: // 101 Step 3
      analogWrite(PWM_U, 0);   digitalWrite(IN_U, LOW);
      analogWrite(PWM_V, PWM); digitalWrite(IN_V, HIGH);
      analogWrite(PWM_W, PWM); digitalWrite(IN_W, LOW);
      break;

    case 6: // 011 Step 1
      analogWrite(PWM_U, PWM); digitalWrite(IN_U, HIGH);
      analogWrite(PWM_V, PWM); digitalWrite(IN_V, LOW);
      analogWrite(PWM_W, 0);   digitalWrite(IN_W, LOW);

      break;

    default:
      // turn off all transistors and read hall sensors:
      analogWrite(PWM_U, 0); digitalWrite(IN_U, LOW);
      analogWrite(PWM_V, 0); digitalWrite(IN_V, LOW);
      analogWrite(PWM_W, 0); digitalWrite(IN_W, LOW);
      // Read Hall sensors
      Hall_State = HallSensorsRead();
  }
}

// How to change the PWM frequency on Arduino https://www.luisllamas.es/en/change-pwm-frequency-arduino/
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
