/*
 * author Jungpyo Lee: <jungpyolee@berkeley.edu> (c.)
 * creation date : Jan. 20. 2022
 * last update : Jul. 07. 2022
 * version 1.1
 * changed: adding wrist mode 2, where wrist angle is used as similar with joystick
 *          FSM inside wrist angle control mode
 * project : Dorsal Grasper v1.1
 * motivation : to change MCU from nucleo-32 to ESP32
 */

// Include headers --------------------------------------
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_VL6180X.h>
#include <Adafruit_VL53L0X.h>
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
Adafruit_MPU6050 mpu;                     // Create instance of the MPU6060 class
ADS ads;                                  // Create instance of the Angular Displacement Sensor (ADS) class
ESP32Encoder encoder;                     // Create instance of the ESP32 encoder class
//Adafruit_VL6180X vl = Adafruit_VL6180X(); // Create instance of the distance sensor class
Adafruit_VL53L0X vl = Adafruit_VL53L0X();// Create instance of the distance sensor class (vl53l0)

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
  count = encoder.getCount();
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
sensors_event_t a, g, temp;                     // mpu6050
uint16_t distance, vl_status;                    // vl6080x & vl53l0x
VL53L0X_RangingMeasurementData_t measure;       // vl53l0x
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
  pinMode(BTN_G, INPUT);
  pinMode(BTN_R, INPUT);
  pinMode(TC, INPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_R, OUTPUT);
  pinMode(JOYSTICK_L, INPUT);
  pinMode(JOYSTICK_R, INPUT);
  pinMode(FOOTPEDAL, INPUT);

  // wait for serial port to open on native usb devices
  while (!Serial)
  {
    delay(1);
  }

  // initialize sensor
//  vl6180x_init();
  vl53l0x_init();
//  mpu6050_init();
  encoder_init();
  ads_init();

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
  timerAlarmWrite(timer1, 50000, true);         // 100000 * 1 us = 50 ms, autoreload true (20 Hz)
  timerAlarmEnable(timer1);                     // enable timer0
  timerStop(timer1);

  digitalWrite(LED_G, HIGH);
  digitalWrite(LED_R, HIGH);
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
    timerStop(timer0);
    timerStop(timer1);
    byte incoming = Serial.read();

    if (incoming == 'c')
    {
      digitalWrite(LED_G, HIGH);
      digitalWrite(LED_R, LOW);
      state = CALIBRATION;
      calibrate_state = LOW;
    }
    if (incoming == 'j')
    {
      digitalWrite(LED_G, HIGH);
      digitalWrite(LED_R, LOW);
      state = JOYSTICK_MODE;
      timerRestart(timer0);
      timerRestart(timer1);
    }
    if (incoming == 'w')
    {
      digitalWrite(LED_R, HIGH);
      digitalWrite(LED_G, LOW);
      state = WRIST_MODE;
      timerRestart(timer0);
      timerRestart(timer1);
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
      portENTER_CRITICAL(&timerMux0);
      timer0_check = false;
      portEXIT_CRITICAL(&timerMux0);
      get_DATA();
      print_DATA();
    }
    if (timer1_check)
    {
      portENTER_CRITICAL(&timerMux1);
      timer1_check = false;
      portEXIT_CRITICAL(&timerMux1);
      joystick_MODE();
    }
    if (button_BOTH())
    {
      motor_STOP();
      timerStop(timer0);
      timerStop(timer1);
      digitalWrite(LED_G, HIGH);
      digitalWrite(LED_R, HIGH);
      state = INITIALIZATION;
    }
    if (incoming == 'i')
    {
      motor_STOP();
      timerStop(timer0);
      timerStop(timer1);
      digitalWrite(LED_G, HIGH);
      digitalWrite(LED_R, HIGH);
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
      portENTER_CRITICAL(&timerMux0);
      timer0_check = false;
      portEXIT_CRITICAL(&timerMux0);
      get_DATA();
      print_DATA();
    }
    if (timer1_check)
    {
      portENTER_CRITICAL(&timerMux1);
      timer1_check = false;
      portEXIT_CRITICAL(&timerMux1);
      wrist_MODE2();
    }
    if (button_BOTH())
    {
      motor_STOP();
      timerStop(timer0);
      timerStop(timer1);
      digitalWrite(LED_G, HIGH);
      digitalWrite(LED_R, HIGH);
      state = INITIALIZATION;
    }
    if (incoming == 'i')
    {
      motor_STOP();
      timerStop(timer0);
      timerStop(timer1);
      digitalWrite(LED_G, HIGH);
      digitalWrite(LED_R, HIGH);
      state = INITIALIZATION;
    }
    if (incoming == 'r')
    {
      encoder_count = 0;
    }
    wrist_MODE2();
    break;
  }
  }
}

// Setup functions --------------------------------------
bool vl6180x_init()
{
  Serial.println("Adafruit VL6180x test!");
  if (!vl.begin())
  {
    Serial.println("Failed to find VL6189x sensor");
    while (1)
      ;
  }
  Serial.println("Sensor found!");
  Serial.println("");
  vl.startRangeContinuous(50);
  delay(100);
}

bool vl53l0x_init()
{
  Serial.println("Adafruit VL53L0X test");
  if (!vl.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  // power 
  Serial.println(F("VL53L0X API Simple Ranging example\n\n")); 
  vl.startRangeContinuous(50);
  delay(100);
}
 
bool mpu6050_init()
{
  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange())
  {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange())
  {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth())
  {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}

bool encoder_init()
{
  Serial.println("Motor encoder Initiation");

  ESP32Encoder::useInternalWeakPullResistors = UP; // Enable the weak pull up resistors
  encoder.attachHalfQuad(E2, E1);                 // Attache pins for use as encoder pins
  encoder.setCount(0);                             // set starting count value after attaching

  Serial.println("");
  delay(100);
}

bool ads_init()
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
bool button_GREEN()
{
  if (digitalRead(BTN_G) == true && digitalRead(BTN_R) == false)
  {
    delay(50);
    if (digitalRead(BTN_G) == true && digitalRead(BTN_R) == false)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    return false;
  }
}

bool button_RED()
{
  if (digitalRead(BTN_G) == false && digitalRead(BTN_R) == true)
  {
    delay(50);
    if (digitalRead(BTN_G) == false && digitalRead(BTN_R) == true)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    return false;
  }
}

bool button_BOTH()
{
  if (digitalRead(BTN_G) == true && digitalRead(BTN_R) == true)
  {
    delay(50);
    if (digitalRead(BTN_G) == true && digitalRead(BTN_R) == true)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    return false;
  }
}

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
  if (ads.available())
  {
    angle = ads.getX();
  }
  //  Serial.print("Acceleration and Gyro: ");
  //  Serial.println(millis()-elapsed_time);

  // t1 = millis();
  // Distance
//  distance = vl.readRangeResult(); // We need to use readRangeResult with continuous reading mode (readRange function has while loop inside it)
    distance = 0;
//  vl.rangingTest(&measure, false);
//  if (measure.RangeStatus !=4)
//  {
//    distance = measure.RangeMilliMeter;
//  }
  // Serial.print("Distnace: ");
  // Serial.println(millis()-t1);

  // Encoder count
  // t2 = millis();
  encoder_count += count;
  motor_speed = count;
  motor_acc = motor_speed - motor_speed_prev;
  motor_speed_prev = motor_speed;
  // Serial.print("Encoder: ");
  // Serial.println(millis()-t2);

  // Joystick
  t3 = millis();
  j_L = digitalRead(JOYSTICK_L);
  j_R = digitalRead(JOYSTICK_R);

  // Pedal
  pedal = digitalRead(FOOTPEDAL);
  // Serial.print("Joystick & pedal: ");
  // Serial.println(millis()-t3);

  // Accecleration and Gyro
  // t4 = millis();
//  mpu.getEvent(&a, &g, &temp);
  // Serial.print("Acceleration and Gyro: ");
  // Serial.println(millis()-t4);

  // Temperature
  // t5 = millis();
//  temperature = get_temperature();
  // Serial.print("temperature: ");
  // Serial.println(millis()-t5);
}

void print_DATA()
{
//  if (state == WRIST_MODE & state2 == IDLE)
//  {
//    Serial.println("IDLE");
//  }
//  else if (state == WRIST_MODE & state2 == CLOSING)
//  {
//    Serial.println("CLOSING");
//  }
//  else if (state == WRIST_MODE & state2 == OPENING)
//  {
//    Serial.println("OPENING");
//  }
//  else if (state == WRIST_MODE & state2 == GRASPING)
//  {
//    Serial.println("GRASPING");
//  }
  if (state == INITIALIZATION)
  {
    Serial.println("i");
    return;
  }
  else if (state == JOYSTICK_MODE)
  {
    Serial.print("j, ");
  }
  else if (state == WRIST_MODE)
  {
    Serial.print("w, ");
  }
  Serial.print(elapsed_time);
  Serial.print(", ");
  Serial.print(angle);
  Serial.print(", ");
  Serial.print(distance);
  Serial.print(", ");
  Serial.print(encoder_count);
  Serial.print(", ");
  Serial.print(motor_speed);
  Serial.print(", ");
  Serial.print(motor_acc);
  Serial.print(", ");
  Serial.print(j_L);
  Serial.print(", ");
  Serial.print(j_R);
  Serial.print(", ");
  Serial.print(pedal);
  Serial.print(", ");
  Serial.print(0);
  Serial.print(", ");
  Serial.print(0);
  Serial.print(", ");
  Serial.print(0);
  Serial.print(", ");
  Serial.print(0);
  Serial.print(", ");
  Serial.print(0);
  Serial.print(", ");
  Serial.print(0);
  Serial.print(", ");
  Serial.println(temperature);
  
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

// AD8495 (Thermal courple) functions
float get_temperature()
{
  float AREF = 3.3;
  int ADC_RESOLUTION = 12;
  float reading, voltage;

  reading = analogRead(TC);
  voltage = reading * (AREF / (pow(2, ADC_RESOLUTION) - 1));
  return (voltage - 1.25) / 0.005;
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
//    else
//    {
//      grasp_count = encoder_count;
//      int target_count = map(angle, OFF_ANGLE, ON_ANGLE, grasp_count + 50, grasp_count);
//      int duty = wrist_PID(encoder_count, target_count);
//
//      if (duty > 0)
//      {
//        ledcWrite(pwmChannel_1, duty);
//        ledcWrite(pwmChannel_2, LOW);
//        digitalWrite(LED_PIN, HIGH);
//      }
//      else if (duty < 0)
//      {
//        ledcWrite(pwmChannel_1, LOW);
//        ledcWrite(pwmChannel_2, -duty);
//        digitalWrite(LED_PIN, HIGH);
//      }
//      else if (duty == 0)
//      {
//        ledcWrite(pwmChannel_1, LOW);
//        ledcWrite(pwmChannel_2, LOW);
//      }
//    }
    break;
  }

  default:
    break;
  }
}
