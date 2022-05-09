#include "HX711.h"
#include <ESP32Encoder.h>

// Define pins ------------------------------------------
// Botton row:
#define MOTOR_A 25    // Motor output A (A0 in ESP32)
#define MOTOR_B 26    // Motor output B (A1 in ESP32)
#define BTN_G 34      // Button green input (A2 in ESP32)
#define BTN_R 39      // Button red input (A3 in ESP32)
#define JOYSTICK_L 36 // Joystick left input (A4 in ESP32)
#define JOYSTICK_R 4  // Joystick right input (A5 in ESP32)
#define FOOTPEDAL 21  // Footpedal switch input (21 in ESP32, it needs pull-down resistor)
// Top row:
#define LED_PIN 13  // ESP32's built-in led pin
#define TEMP_PIN 12 // Should not use this pin to work appropriately
#define E1 27       // Encoder pin1
#define E2 33       // Encoder pin2
#define LOADCELL_DOUT_PIN 15    // Loadcell 1 data
#define LOADCELL_SCK_PIN 32    // Loadcell 1 clock
#define MOTOR_EN 14       // Motor enable pin
#define LOADCELL_DOUT_PIN_2 22      // Loadcell 2 data
#define LOADCELL_SCK_PIN_2 23      // Loadcell 2 clock

// Define states ----------------------------------------
#define INITIALIZATION 1 // Initialization state
#define MANUAL_MODE 2  // Joystick control mode
#define AUTOMATIC_MODE 3     // Force control by tension of tendon
#define AUTOMATIC_MODE2 4    // Force control by wrist reaction of load cell
#define MOTOR_DISABLED 5     // Motor disabled mode to not feeding PWM

// Create instance --------------------------------------
HX711 scale;                              // Create instance for loadcell (palm)
HX711 scale2;                             // Create instance for loadcell (tendon)
ESP32Encoder encoder;                     // Create instance of the ESP32 encoder class

// Setup interrupt variables ----------------------------
volatile int count = 0;             // encoder count


// Setup interrupt functions ----------------------------


// Setup variables --------------------------------------
const int freq = 20000;
const int pwmChannel_1 = 1;
const int pwmChannel_2 = 2;
const int resolution = 8; // PWM value from 0 to 255)
const int MAX_PWM_VOLTAGE = 200; // too fast
const int NOM_PWM_VOLTAGE = 150;
const int JOYSTICK_PWM = 200;
const int MAX_EN = 850; // encoder value in fully closed finger
const int MAX_ANGLE = 60; // maximum angle of the wrist
const int MIN_ANGLE = 20; // minimum angle of the wrist
const int Kp = 10;      // P gain for tendon
const float Ki = 0.9;     // I gain for tendon
const int Kd = 0;       // D gain for tendon
const int Kp2 = 500;      // P gain for wrist
const float Ki2 = 45;     // I gain for wrist
const int Kd2 = 0;       // D gain for wrist
int duty = 0;
float Err = 0;

// Record variables -------------------------------------
int state = INITIALIZATION;
unsigned long elapsed_time;             // elapsed time
int encoder_count;                      // position of the motor
uint8_t j_L, j_R;                       // joystick left, right, and pedal input
float palm_raw;                         // loadcell value from palm [g]
float palm;                             // loadcell value from palm [N]
float palm_desire_max = 1.92;           // maximum wrist strengh [Nm]
float palm_desire = 0.5;                // desired wrist reaction for the experiment [Nm]
float palm_current = 0;                 // current_wrist reaction [Nm]
float tendon_raw;                       // tendon loadcell data
float tendon;                           // tendon loadcell data converted to N
float tendon_desire = 50;               // tension of the tendon limits 60 N
int contactPoint;

void setup() {
  Serial.begin(115200);
  pinMode(JOYSTICK_L, INPUT);
  pinMode(JOYSTICK_R, INPUT);
  pinMode(MOTOR_EN, OUTPUT);

  // Palm loadcell initialization
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(476.8);
  scale.tare();

  // Tendon loadcell initialization
  scale2.begin(LOADCELL_DOUT_PIN_2, LOADCELL_SCK_PIN_2);
  scale2.set_scale(239);
  scale2.tare();

  // encoder initialization
  encoder_init();

  // initialize motor
  Serial.println("Motor PWM Initiation");

  // configure LED PWM functionalitites
  ledcSetup(pwmChannel_1, freq, resolution);
  ledcSetup(pwmChannel_2, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(MOTOR_A, pwmChannel_1);
  ledcAttachPin(MOTOR_B, pwmChannel_2);
  delay(100);
}

void loop() {

switch (state)
  {
  case INITIALIZATION:
  {
    motor_STOP();
//    timerStop(timer0);
//    timerStop(timer1);
    byte incoming = Serial.read();

    if (incoming == 'm')
    {
      state = MANUAL_MODE;
    }
    if (incoming == 'a')
    {
      state = AUTOMATIC_MODE;
//      timerRestart(timer0);
//      timerRestart(timer1);
    }
    if (incoming == 'b')
    {
      while (Serial.available() > 0)
       Serial.read();
      state = AUTOMATIC_MODE2;
      Serial.println("Input contact point in mm ");
      while(Serial.available() == 0){}
      contactPoint = Serial.parseInt();
    }
    Serial.println('i');
    delay(100);
    break;
  }
  
  case MANUAL_MODE:
  {
    byte incoming = Serial.read();

    if (incoming == 'i')
    {
      state = INITIALIZATION;
      motor_STOP();
    }
    if (incoming == 'a')
    {
      state = AUTOMATIC_MODE;
    }
    if (incoming == 'b')
    {
      while (Serial.available() > 0)
        Serial.read();
        
      state = AUTOMATIC_MODE2;
      Serial.println("Input contact point in mm ");
      while(Serial.available() == 0){}
      contactPoint = Serial.parseInt();
    }
    if (incoming == 'p')
    {
      state = MOTOR_DISABLED;
    }
    get_DATA();
    print_DATA();
    manual_MODE();
    break;
  }

  case AUTOMATIC_MODE:
  {
    byte incoming = Serial.read();

    if (incoming == 'i')
    {
      state = INITIALIZATION;
      motor_STOP();
    }
    if (incoming == 'b')
    {
      while (Serial.available() > 0)
       Serial.read();
       
      state = AUTOMATIC_MODE2;
      Serial.println("Input contact point in mm ");
      while(Serial.available() == 0){}
      contactPoint = Serial.parseInt();
    }
    if (incoming == 'm')
    {
      state = MANUAL_MODE;
    }
    if (incoming == 'p')
    {
      state = MOTOR_DISABLED;
    }
    get_DATA();
    print_DATA();
    automatic_MODE();
    break;
  }

  case AUTOMATIC_MODE2:
  {
    byte incoming = Serial.read();

    if (incoming == 'i')
    {
      state = INITIALIZATION;
      motor_STOP();
    }
    if (incoming == 'a')
    {
      state = AUTOMATIC_MODE;
    }
    if (incoming == 'm')
    {
      state = MANUAL_MODE;
    }
    if (incoming == 'p')
    {
      state = MOTOR_DISABLED;
    }
    get_DATA();
    print_DATA();
    automatic_MODE2();
    break;
  }

  case MOTOR_DISABLED:
  {
    byte incoming = Serial.read();

    if (incoming == 'i')
    {
      state = INITIALIZATION;
      motor_STOP();
    }
    if (incoming == 'm')
    {
      state = MANUAL_MODE;
    }
    if (incoming == 'a')
    {
      state = AUTOMATIC_MODE;
    }
    if (incoming == 'b')
    {
      while (Serial.available() > 0)
       Serial.read();
      state = AUTOMATIC_MODE2;
      Serial.println("Input contact point in mm ");
      while(Serial.available() == 0){}
      contactPoint = Serial.parseInt();
    }
    get_DATA();
    print_DATA();
    motor_DISALBED();
    break;
  }
}
}


// Initialization functions -----------------------------------
void encoder_init()
{
  Serial.println("Motor encoder Initiation");

  ESP32Encoder::useInternalWeakPullResistors = UP; // Enable the weak pull up resistors
  encoder.attachHalfQuad(E2, E1);                 // Attache pins for use as encoder pins
  encoder.setCount(0);                             // set starting count value after attaching

  Serial.println("");
  delay(100);
}

// General functions -----------------------------------
void get_DATA()
{
  palm_raw = scale.get_units();
  tendon_raw = scale2.get_units();

  // convert palm_raw to Nm units
  palm = 0.001 * 9.81 * palm_raw; // convert to Kg, multiply 9.81 m/s^2, multiply 0.065 m (contact point)
  tendon = 0.001 * 9.81 * tendon_raw;
}

void print_DATA()
{
  Serial.print("Current State: ");
  switch (state)
  {
    case INITIALIZATION:
    {
      Serial.println("initialization");
    }
    case MANUAL_MODE:
    {
      Serial.println("manual mode");
    }
    case AUTOMATIC_MODE:
    {
      Serial.println("automatic mode");
    }
    case AUTOMATIC_MODE2:
    {
      Serial.println("automatic mode 2");
    }
    case MOTOR_DISABLED:
    {
      Serial.println("motor disabled mode");
    }
  }
  Serial.print("Tendon loadcell: ");
  Serial.print(tendon);
  Serial.println("N");
  Serial.print("Palm loadcell: ");
  Serial.print(palm);
  Serial.println("N");
//  Serial.print("Wrist strength: ");
//  Serial.print(palm);
//  Serial.println("Nm");
 
  Serial.print("Error: ");
  Serial.println(Err);
  Serial.print("PWM: ");
  Serial.print(duty);
  Serial.println("");
}

void motor_DISALBED()
{
  digitalWrite(MOTOR_EN, LOW);
  ledcWrite(pwmChannel_1, LOW);
  ledcWrite(pwmChannel_2, LOW);
  digitalWrite(LED_PIN, LOW);
}

void motor_STOP()
{
  digitalWrite(MOTOR_EN, HIGH);
  ledcWrite(pwmChannel_1, LOW);
  ledcWrite(pwmChannel_2, LOW);
  digitalWrite(LED_PIN, LOW);
}

void motor_FORWARD()
{
  digitalWrite(MOTOR_EN, HIGH);
  ledcWrite(pwmChannel_1, JOYSTICK_PWM);
  ledcWrite(pwmChannel_2, LOW);
  digitalWrite(LED_PIN, HIGH);
}

void motor_BACKWARD()
{
  digitalWrite(MOTOR_EN, HIGH);
  ledcWrite(pwmChannel_1, LOW);
  ledcWrite(pwmChannel_2, JOYSTICK_PWM);
  digitalWrite(LED_PIN, HIGH);
}

void manual_MODE()
{
  if (digitalRead(JOYSTICK_L) == true)
  {
    motor_FORWARD();
  }
  else if (digitalRead(JOYSTICK_R) == true)
  {
    motor_BACKWARD();
  }
  else
  {
    motor_STOP();
  }
}

void automatic_MODE()
{
  duty = PID(tendon, tendon_desire);

  if (duty > 0)
  {
    ledcWrite(pwmChannel_1, duty);
    ledcWrite(pwmChannel_2, LOW);
    digitalWrite(LED_PIN, HIGH);
  }
  else if (duty < 0)
  {
    ledcWrite(pwmChannel_1, LOW);
    ledcWrite(pwmChannel_2, -duty);
    digitalWrite(LED_PIN, HIGH);
  }
  else if (duty == 0)
  {
    ledcWrite(pwmChannel_1, LOW);
    ledcWrite(pwmChannel_2, LOW);
  }
}

void automatic_MODE2() // wrist angle control
{
  palm_current = palm * contactPoint * 0.001; // current wrist reaction Nm
//  Serial.print("palm_current: ");
//  Serial.println(palm_current);
//  Serial.print("palm_desire: ");
//  Serial.println(palm_desire);
  
  duty = PID2(palm_current, palm_desire);

  if (duty > 0)
  {
    ledcWrite(pwmChannel_1, duty);
    ledcWrite(pwmChannel_2, LOW);
    digitalWrite(LED_PIN, HIGH);
  }
  else if (duty < 0)
  {
    ledcWrite(pwmChannel_1, LOW);
    ledcWrite(pwmChannel_2, -duty);
    digitalWrite(LED_PIN, HIGH);
  }
  else if (duty == 0)
  {
    ledcWrite(pwmChannel_1, LOW);
    ledcWrite(pwmChannel_2, LOW);
  }
}

int PID(int current, int target)
{
  static int LastErr;
  static float pwm, SumErr;

  Err = target - current;
  SumErr += Err;
  // Serial.println(SumErr);
  pwm = Kp * Err + Ki * SumErr + Kd * (Err - LastErr);
  LastErr = Err;

  if (pwm > MAX_PWM_VOLTAGE)
  {
    pwm = MAX_PWM_VOLTAGE;
    SumErr = SumErr - Err; // anti-windup
  }
  else if (pwm < -MAX_PWM_VOLTAGE)
  {
    pwm = -MAX_PWM_VOLTAGE;
    SumErr = SumErr - Err; // anti-windup
  }
  // Serial.println(pwm);
  return (int)pwm;
}


int PID2(float current, float target)
{
//  Serial.print("palm_current: ");
//  Serial.println(current);
//  Serial.print("palm_desire: ");
//  Serial.println(target);
  static int LastErr;
  static float pwm, SumErr;

  Err = target - current;
  SumErr += Err;
  // Serial.println(SumErr);
  pwm = Kp2 * Err + Ki2 * SumErr + Kd2 * (Err - LastErr);
  LastErr = Err;

  if (pwm > MAX_PWM_VOLTAGE)
  {
    pwm = MAX_PWM_VOLTAGE;
    SumErr = SumErr - Err; // anti-windup
  }
  else if (pwm < -MAX_PWM_VOLTAGE)
  {
    pwm = -MAX_PWM_VOLTAGE;
    SumErr = SumErr - Err; // anti-windup
  }
  // Serial.println(pwm);
  return (int)pwm;
}
