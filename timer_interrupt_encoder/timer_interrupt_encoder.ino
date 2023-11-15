/*
 * author Jungpyo Lee: <jungpyolee@berkeley.edu> (c.)
 * creation date : Jan. 20. 2022
 * last update : Nov. 09. 2023
 * version 1.0
 * changed: change code to check timer interrupt
 * project : Dorsal Grasper 2.0
 * motivation : MEng project (tutorial device)
 */

// Include headers --------------------------------------
#include <Wire.h>
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
#define TEST_MODE 4      // Mode for testing systems

// Define states for the wrist angle mode ---------------
#define IDLE 4          // Idle state (PWM = 0)
#define CLOSING 5       // Finger closing state (PWM = positive)
#define OPENING 6       // Finger opeing state (PWM = negative)
#define GRASPING 7      // Grrasping mode, which finger follows wrist

// Create instance --------------------------------------
ESP32Encoder encoder;                     // Create instance of the ESP32 encoder class

// Setup interrupt variables ----------------------------
volatile int count = 0;             // encoder count for speed
volatile bool timer0_check = false; // check timer interrupt 0
volatile bool timer1_check = false; // check timer interrupt 1
hw_timer_t *timer0 = NULL;
hw_timer_t *timer1 = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux1 = portMUX_INITIALIZER_UNLOCKED;


// Setup interrupt functions ----------------------------
void IRAM_ATTR onTime0()
{ // this can be used for constant data acquisition
  portENTER_CRITICAL_ISR(&timerMux0);
  timer0_check = true; // the function to be called when timer interrupt is triggered
  portEXIT_CRITICAL_ISR(&timerMux0);
}

void IRAM_ATTR onTime1()
{ // this can be used for constant data acquisition
  portENTER_CRITICAL_ISR(&timerMux1);
  count = -encoder.getCount();
  encoder.clearCount();
  timer1_check = true; // the function to be called when timer interrupt is triggered
  portEXIT_CRITICAL_ISR(&timerMux1);
}

// Setup variables --------------------------------------
const int freq = 20000;
const int pwmChannel_1 = 1;
const int pwmChannel_2 = 2;
const int resolution = 8; // PWM value from 0 to 255)
const int MAX_PWM_VOLTAGE = 200; // too fast
const int NOM_PWM_VOLTAGE = 150;
const int TEST_PWM = 200; // motor PWM value for the joystick mode
const int MAX_EN = 1200; // encoder value in fully closed finger
bool calibrate_state;

// Record variables -------------------------------------
int state = INITIALIZATION;                     // state for the main loop
unsigned long elapsed_time;
int encoder_count;                              // position of the motor
int motor_speed = 0;                            // speed of the motor (unit is count)
int motor_speed_prev;                           // to calculate motor acceleration
int motor_acc;                                  // acceleration of the motor (unit is count)
bool motor_status = false;                      // motor_status for status change from closing to grasping

void setup()
{
  Serial.begin(115200);
  Serial.println("Start Setup");

  // wait for serial port to open on native usb devices
  while (!Serial)
  {
    delay(1);
  }

  // initialize timer; timer0 = data acquisition, timer1 = motor operation
  Serial.println("Timer0 initializatoin");
  timer0 = timerBegin(0, 80, true);             // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer0, &onTime0, true); // edge (not level) triggered
  timerAlarmWrite(timer0, 50000, true);        // 50000 * 1 us = 50 ms, autoreload true (20 Hz)
  timerAlarmEnable(timer0);                     // enable timer0
  timerStop(timer0);

  Serial.println("Timer1 initializatoin");
  timer1 = timerBegin(1, 80, true);             // timer 1, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer1, &onTime1, true); // edge (not level) triggered
  timerAlarmWrite(timer1, 50000, true);        // 50000 * 1 us = 50 ms, autoreload true (20 Hz)
  timerAlarmEnable(timer1);                     // enable timer0
  timerStop(timer1);

  encoder_init();

  Serial.println("");
  Serial.println("Setup done");
}

void loop()
{
  switch (state)
  {
  case INITIALIZATION:
  {
    byte incoming = Serial.read();

    if (incoming == 't')
    {
      state = TEST_MODE;
      timerWrite(timer0, 0);
      timerStart(timer0);
      timerWrite(timer1, 0);
      timerStart(timer1);
    }
    Serial.println('i');
    delay(100);
    break;
  }

  case TEST_MODE:
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
    }
    if (incoming == 'i')
    {
      timerStop(timer0);
      timerStop(timer1);
      state = INITIALIZATION;
    }
    break;
  }
  }
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
  // vl.startRange();
  // distance = vl.readRange();
  encoder_count += count;
  motor_speed = count;
  motor_acc = motor_speed - motor_speed_prev;
  motor_speed_prev = motor_speed;
}

void print_DATA()
{
  Serial.print(elapsed_time);
  Serial.print(", ");
  // Serial.print(distance);
  // Serial.print(", ");
  Serial.print(encoder_count);
  Serial.print(", ");
  Serial.print(motor_speed);
  Serial.print(", ");
  Serial.println(motor_acc);
}
