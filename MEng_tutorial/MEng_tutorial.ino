/*
 * author Jungpyo Lee: <jungpyolee@berkeley.edu> (c.)
 * creation date : Jan. 20. 2022
 * last update : Nov. 03. 2023
 * version 1.0
 * changed: change code for the tutorial device (wrist angle control mode, no IMU, no thermocouple, use LV_6180X)
 * project : Dorsal Grasper 2.0
 * motivation : MEng project (tutorial device)
 */

// Include headers --------------------------------------
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_VL6180X.h>
#include <SparkFun_Displacement_Sensor_Arduino_Library.h>
#include <ESP32Encoder.h>

// Define pins ------------------------------------------
// Botton row:
#define MOTOR_A 26    // Motor output A (A0 in ESP32)
#define MOTOR_B 25    // Motor output B (A1 in ESP32)
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
#define LED_G 15    // LED green output
#define LED_R 32    // LED red output
#define TC 14       // Thermal couple analog input
#define SCL 22      // i2c scl pin
#define SDA 23      // i2c sda pin

// Define states ----------------------------------------
#define CALIBRATION 0    // Calibration state
#define INITIALIZATION 1 // Initialization state
#define JOYSTICK_MODE 2  // Joystick control mode
#define WRIST_MODE 3     // Wrist angle control mode

// Define states for the wrist angle mode ---------------
#define IDLE 4          // Idle state (PWM = 0)
#define CLOSING 5       // Finger closing state (PWM = positive)
#define OPENING 6       // Finger opeing state (PWM = negative)
#define GRASPING 7      // Grrasping mode, which finger follows wrist

// Create instance --------------------------------------
ADS ads;                                  // Create instance of the Angular Displacement Sensor (ADS) class
ESP32Encoder encoder;                     // Create instance of the ESP32 encoder class
Adafruit_VL6180X vl = Adafruit_VL6180X(); // Create instance of the distance sensor class

// Setup interrupt variables ----------------------------
volatile int count = 0;             // encoder count for speed
volatile bool timer0_check = false; // check timer interrupt 0
volatile bool timer1_check = false; // check timer interrupt 1
hw_timer_t *timer0 = NULL;
hw_timer_t *timer1 = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux1 = portMUX_INITIALIZER_UNLOCKED;
volatile bool button_G_press = false;
volatile bool button_R_press = false;

// Setup interrupt functions ----------------------------
void IRAM_ATTR onTime0()
{ // this can be used for constant data acquisition
  portENTER_CRITICAL_ISR(&timerMux0);
  timer0_check = true; // the function to be called when timer interrupt is triggered
  portEXIT_CRITICAL_ISR(&timerMux0);
}

void IRAM_ATTR onTime1()
{ // this can be used for the motor operation
  portENTER_CRITICAL_ISR(&timerMux1);
  count = -encoder.getCount();
  encoder.clearCount();
  timer1_check = true; // the function to be called when timer interrupt is triggered
  portEXIT_CRITICAL_ISR(&timerMux1);
}

void IRAM_ATTR isr0()
{ // the function to be called when interrupt is triggered
  button_G_press = true;
}

void IRAM_ATTR isr1()
{ // the function to be called when interrupt is triggered
  button_R_press = true;
}

// Setup variables --------------------------------------
const int freq = 20000;
const int pwmChannel_1 = 1;
const int pwmChannel_2 = 2;
const int resolution = 8; // PWM value from 0 to 255)
const int MAX_PWM_VOLTAGE = 200; // too fast
const int NOM_PWM_VOLTAGE = 150;
const int JOYSTICK_PWM = 200; // motor PWM value for the joystick mode
const int WRIST_PWM = 200; // motor PWM value for the wrist angle mode
const int MAX_EN = 1200; // encoder value in fully closed finger
const int MAX_ANGLE = 45; // maximum angle of the wrist
const int MIN_ANGLE = 10; // minimum angle of the wrist
const int ON_ANGLE = 20; // on angle to close the finger
const int OFF_ANGLE = 10; // off angle to open the inger
const int HIGH_VELOCITY = 50; // High threshold velocity
const int LOW_VELOCITY = 40; // Low threshold velocity
const int DIST_THRESHOLD = 60; // Distance threshold for the wrist angle control
const int Kp = 2;      // P gain
const int Ki = 0.1;     // I gain
const int Kd = 0;       // D gain
bool calibrate_state;

// Record variables -------------------------------------
int state = INITIALIZATION;                     // state for the main loop
int state2 = IDLE;                              // state for the wristn angle mode
uint16_t distance, vl_status;                    // vl6080x & vl53l0x
unsigned long elapsed_time, t1, t2, t3, t4, t5; // elapsed time
float temperature;                              // temperature from ad8405
int encoder_count;                              // position of the motor
int grasp_count;                                // position of the motor when the grasping mode starts
int motor_speed = 0;                            // speed of the motor (unit is count)
int motor_speed_prev;                           // to calculate motor acceleration
int motor_acc;                                  // acceleration of the motor (unit is count)
bool motor_status = false;                      // motor_status for status change from closing to grasping
uint8_t j_L, j_R, pedal;                        // joystick left, right, and pedal input
float angle;                                    // wrist angle from ads

void setup()
{
  Serial.begin(115200);
  Wire.begin(SDA, SCL); // setup for i2c pins of ESP32, sda= GPIO_23 /scl= GPIO_22
  Serial.println("Start Setup");
  pinMode(JOYSTICK_L, INPUT);
  pinMode(JOYSTICK_R, INPUT);

  // wait for serial port to open on native usb devices
  while (!Serial)
  {
    delay(1);
  }

  // initialize sensor
  // vl6180x_init();
  encoder_init();
  // ads_init();

  // initialize motor
  Serial.println("Motor PWM Initiation");

  // configure LED PWM functionalitites
  ledcSetup(pwmChannel_1, freq, resolution);
  ledcSetup(pwmChannel_2, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(MOTOR_A, pwmChannel_1);
  ledcAttachPin(MOTOR_B, pwmChannel_2);
  delay(100);

  // initialize timer; timer0 = data acquisition, timer1 = motor operation
  Serial.println("Timer0 initializatoin");
  timer0 = timerBegin(0, 80, true);             // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer0, &onTime0, true); // edge (not level) triggered
  timerAlarmWrite(timer0, 50000, true);        // 50000 * 1 us = 50 ms, autoreload true (20 Hz)
  timerAlarmEnable(timer0);                     // enable timer0
  timerStop(timer0);

  Serial.println("Timer1 initializatoin");
  timer1 = timerBegin(1, 80, true);             // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer1, &onTime1, true); // edge (not level) triggered
  timerAlarmWrite(timer1, 50000, true);        // 50000 * 1 us = 50 ms, autoreload true (20 Hz)
  timerAlarmEnable(timer1);                     // enable timer0
  timerStop(timer1);

  Serial.println("");
  Serial.println("Setup done");
}

void loop()
{
  switch (state)
  {
  case INITIALIZATION:
  {
    motor_STOP();
    byte incoming = Serial.read();

    if (incoming == 'c')
    {
      state = CALIBRATION;
      calibrate_state = LOW;
    }
    if (incoming == 'j')
    {
      state = JOYSTICK_MODE;
      timerWrite(timer0, 0);
      timerStart(timer0);
      timerWrite(timer1, 0);
      timerStart(timer1);
      // Serial.println("hi");
    }
    if (incoming == 'w')
    {
      state = WRIST_MODE;
      timerWrite(timer0, 0);
      timerStart(timer0);
      timerWrite(timer1, 0);
      timerStart(timer1);
    }
    if (incoming == 'r')
    {
      encoder_count = 0;
    }
    Serial.println('i');
    delay(100);
    break;
  }

  case CALIBRATION:
  {
    calibrate();
    if (calibrate_state = HIGH)
    {
      state = INITIALIZATION;
    }
    break;
  }

  case JOYSTICK_MODE:
  {
    byte incoming = Serial.read();
    if (timer0_check)
    {
      // Serial.println("timer 0");
      portENTER_CRITICAL(&timerMux0);
      timer0_check = false;
      portEXIT_CRITICAL(&timerMux0);
      get_DATA();
      print_DATA();
    }
    if (timer1_check)
    {
      // Serial.println("timer 1");
      portENTER_CRITICAL(&timerMux1);
      timer1_check = false;
      portEXIT_CRITICAL(&timerMux1);
      joystick_MODE();
    }
    if (incoming == 'i')
    {
      motor_STOP();
      timerStop(timer0);
      timerStop(timer1);
      state = INITIALIZATION;
    }
    if (incoming == 'r')
    {
      encoder_count = 0;
    }
    joystick_MODE();
    break;
  }

  case WRIST_MODE:
  {
    byte incoming = Serial.read();
    if (timer0_check)
    {
      // Serial.println("timer 0");
      portENTER_CRITICAL(&timerMux0);
      timer0_check = false;
      portEXIT_CRITICAL(&timerMux0);
      get_DATA();
      print_DATA();
    }
    if (timer1_check)
    {
      // Serial.println("timer 1");
      portENTER_CRITICAL(&timerMux1);
      timer1_check = false;
      portEXIT_CRITICAL(&timerMux1);
      wrist_MODE3();
    }
    if (incoming == 'i')
    {
      motor_STOP();
      timerStop(timer0);
      timerStop(timer1);
      state = INITIALIZATION;
    }
    if (incoming == 'r')
    {
      encoder_count = 0;
    }
    // wrist_MODE2();
    // Serial.println('w');
    break;
  }
  }
}

// Setup functions --------------------------------------
void vl6180x_init()
{
  Serial.println("Adafruit VL6180x test!");
  vl.begin();
  delay(100);
  // if (!vl.begin())
  // {
  //   Serial.println("Failed to find VL6180x sensor");
  //   while (1)
  //     ;
  // }
  Serial.println("Sensor found!");
  Serial.println("");
  vl.startRangeContinuous(50);
  delay(100);
}

void encoder_init()
{
  Serial.println("Motor encoder Initiation");

  ESP32Encoder::useInternalWeakPullResistors = UP; // Enable the weak pull up resistors
  encoder.attachHalfQuad(E2, E1);                 // Attache pins for use as encoder pins
  encoder.setCount(0);                             // set starting count value after attaching

  Serial.println("");
  delay(100);
}

void ads_init()
{
  Serial.println("SparkFun Displacement Sensor Initiation");

  if (ads.begin() == false)
  {
    Serial.println(F("No bending sensor detected. Check wiring. Freezing..."));
    while (1)
      ;
  }
  delay(150);
}

// Event checkers --------------------------------------


// Service routines ------------------------------------
void get_DATA()
{
  /*
   * int state = INITIALIZATION;
   * sensors_event_t a, g, temp; // mpu6050
   * uint8_t distance, vl_status; // vl6080x
   * unsigned long elapsed_time; // elapsed time
   * float temperature; // temperature from ad8405
   * int encoder_count; // position of the motor
   * uint8_t j_L, j_R, pedal; // joystick left, right, and pedal input
   * float angle; // wrist angle from ads
   */
  // Elapsed time
  elapsed_time = millis();

  // Wrist angle
  // if (ads.available())
  // {
  //   angle = ads.getX();
  // }

  // t1 = millis();
  // Distance
//  distance = vl.readRangeResult(); // We need to use readRangeResult with continuous reading mode (readRange function has while loop inside it)
  // vl.startRange();
  // distance = vl.readRange();

  // Encoder count
  encoder_count += count;
  motor_speed = count;
  motor_acc = motor_speed - motor_speed_prev;
  motor_speed_prev = motor_speed;

  // Joystick
  j_L = digitalRead(JOYSTICK_L);
  j_R = digitalRead(JOYSTICK_R);

}

void print_DATA()
{
  if (state == WRIST_MODE)
  {
    Serial.print("w, ");
  } else if (state == JOYSTICK_MODE)
  {
    Serial.print("j, ");
  }
  Serial.print(elapsed_time);
  Serial.print(", ");
  // Serial.print(angle);
  // Serial.print(", ");
  // Serial.print(distance);
  // Serial.print(", ");
  Serial.print(encoder_count);
  Serial.print(", ");
  Serial.print(motor_speed);
  Serial.print(", ");
  Serial.print(motor_acc);
  Serial.print(", ");
  Serial.print(j_L);
  Serial.print(", ");
  Serial.println(j_R);
}

// General functions -----------------------------------
void motor_STOP()
{
  ledcWrite(pwmChannel_1, LOW);
  ledcWrite(pwmChannel_2, LOW);
  digitalWrite(LED_PIN, LOW);
}

void motor_FORWARD()
{
  ledcWrite(pwmChannel_1, JOYSTICK_PWM);
  ledcWrite(pwmChannel_2, LOW);
  digitalWrite(LED_PIN, HIGH);
}

void motor_BACKWARD()
{
  ledcWrite(pwmChannel_1, LOW);
  ledcWrite(pwmChannel_2, JOYSTICK_PWM);
  digitalWrite(LED_PIN, HIGH);
}


void calibrate()
{
  Serial.println("c");

  while (Serial.available() > 0)
    Serial.read(); // Flush all characters
  // Serial.println(F("Press a key when the wrist is 0 degree angle"));
  while (Serial.available() == 0)
  {
    ads.available();
    delay(10); // Wait for user to press character
    Serial.println("c");
  }

  ads.calibrateZero(); // Call when sensor is straight on both axis

  Serial.println("0");

  while (Serial.available() > 0)
    Serial.read(); // Flush all characters
  // Serial.println(F("Good. Now press a key when the wrist is bent at 45 degrees (extension)."));
  while (Serial.available() == 0)
  {
    ads.available();
    delay(10); // Wait for user to press character
    Serial.println("0");
  }

  ads.calibrateX45(); // Call when sensor is 45 degrees on X axis

  Serial.println("45");

  calibrate_state = HIGH;
}

void joystick_MODE()
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


void wrist_MODE()
{
  int target_count;
  if (angle <= MIN_ANGLE)
  {
    target_count = 0;
  }
  else if (angle > MAX_ANGLE)
  {
    target_count = MAX_EN;
  }
  else if (angle > MIN_ANGLE && angle <= MAX_ANGLE)
  {
    target_count = map(angle, MIN_ANGLE, MAX_ANGLE, 0, MAX_EN);
  }
  int duty = wrist_PID(encoder_count, target_count);

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

int wrist_PID(int current, int target)
{
  static int LastErr;
  static float pwm, SumErr;

  int Err = target - current;
  SumErr += Err;
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
  return (int)pwm;
}

// Wrist angle control 2 (Jul.07.2022)
void wrist_MODE2()
{
  switch (state2)
  {
  case IDLE:
  {
    motor_STOP();
    if (angle >= ON_ANGLE & distance < DIST_THRESHOLD)
    {
      motor_FORWARD();
      state2 = CLOSING;
    }
    else if (angle < OFF_ANGLE && encoder_count > 0)
    {
      motor_BACKWARD();
      state2 = OPENING;
    }
    break;
  }
  case CLOSING:
  {
    if (motor_speed >= HIGH_VELOCITY)
    {
      motor_status = true;
    }
    if (motor_speed < LOW_VELOCITY & motor_status == true)
    {
      motor_STOP();
      motor_status = false;
      state2 = GRASPING;
    }
    else if (angle < OFF_ANGLE)
    {
      motor_BACKWARD();
      motor_status = false;
      state2 = OPENING;
    }
    break;
  }
  case OPENING:
  {
    if (encoder_count <= 0)
    {
      motor_STOP();
      state2 = IDLE;
    }
    else if (angle >= ON_ANGLE & distance < DIST_THRESHOLD)
    {
      motor_FORWARD();
      state2 = CLOSING;
    }
    break;
  }
  case GRASPING:
  {
    if (angle < OFF_ANGLE)
    {
      motor_BACKWARD();
      state2 = OPENING;
    }
    break;
  }

  default:
    break;
  }
}


// Wrist angle control 3 for MEng team which doesn't include distance sensor(Nov.13.2023)
void wrist_MODE3()
{
  switch (state2)
  {
  case IDLE:
  {
    motor_STOP();
    if (angle >= ON_ANGLE)
    {
      motor_FORWARD();
      state2 = CLOSING;
    }
    else if (angle < OFF_ANGLE && encoder_count > 0)
    {
      motor_BACKWARD();
      state2 = OPENING;
    }
    break;
  }
  case CLOSING:
  {
    if (motor_speed >= HIGH_VELOCITY)
    {
      motor_status = true;
    }
    if (motor_speed < LOW_VELOCITY & motor_status == true)
    {
      motor_STOP();
      motor_status = false;
      state2 = GRASPING;
    }
    else if (angle < OFF_ANGLE)
    {
      motor_BACKWARD();
      motor_status = false;
      state2 = OPENING;
    }
    break;
  }
  case OPENING:
  {
    if (encoder_count <= 0)
    {
      motor_STOP();
      state2 = IDLE;
    }
    else if (angle >= ON_ANGLE)
    {
      motor_FORWARD();
      state2 = CLOSING;
    }
    break;
  }
  case GRASPING:
  {
    if (angle < OFF_ANGLE)
    {
      motor_BACKWARD();
      state2 = OPENING;
    }
    break;
  }

  default:
    break;
  }
}
