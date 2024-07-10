
/*
 * author Jungpyo Lee: <jungpyolee@berkeley.edu> (c.)
 * creation date : May. 22. 2024
 * last update : May. 22. 2024
 * source: MENG_final_code_tinypico
 * version 1.0
 * changed: trim the code for the home test device (wrist angle control mode only)
 * project : Home based evaluation of the Dorsal Grasper
 * motivation : MEng project (tutorial device)
 */

// Include headers --------------------------------------
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <SparkFun_Displacement_Sensor_Arduino_Library.h>
#include <ESP32Encoder.h>
#include <TinyPICO.h>

// Define pins (TinyPICO) ------------------------------------------
#define MOTOR_A 32    // Motor output A
#define MOTOR_B 33    // Motor output B
#define CALIBRATION_BUTTON 15 // for calibration
#define JOYSTICK_BUTTON 27  // for josytick mode (reset finger position)
#define E1 14       // Encoder pin1
#define E2 4       // Encoder pin2
#define SCL 22      // i2c scl pin
#define SDA 21      // i2c sda pin
#define flexSensorPin  25 // flexible sensor pin

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
ESP32Encoder encoder;                     // Create instance of the ESP32 encoder class

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
const int JOYSTICK_PWM = 230; // motor PWM value for the joystick mode
const int WRIST_PWM = 250; // motor PWM value for the wrist angle mode
const int MAX_EN = 1200; // encoder value in fully closed finger
const int MAX_ANGLE = 45; // maximum angle of the wrist
const int MIN_ANGLE = 10; // minimum angle of the wrist
const int ON_ANGLE = 360; // on angle to close the finger
const int OFF_ANGLE = 320; // off angle to open the inger
const int HIGH_VELOCITY = 70; // High threshold velocity
const int LOW_VELOCITY = 65; // Low threshold velocity
bool calibrate_state;

// Record variables -------------------------------------
int state = WRIST_MODE;                     // state for the main loop
int state2 = IDLE;                              // state for the wristn angle mode
//static int lastButtonState = 0;
unsigned long elapsed_time, t1, t2, t3, t4, t5; // elapsed time
int encoder_count;                              // position of the motor
int grasp_count;                                // position of the motor when the grasping mode starts
int motor_speed = 0;                            // speed of the motor (unit is count)
int motor_speed_prev;                           // to calculate motor acceleration
int motor_acc;                                  // acceleration of the motor (unit is count)
bool motor_status = false;                      // motor_status for status change from closing to grasping
float angle;                                    // wrist angle from spectral flexible sensor

TinyPICO tp = TinyPICO();

void setup()
{
  Serial.begin(115200);
  Serial.println("Start Setup");
 
  pinMode(CALIBRATION_BUTTON, INPUT);
  pinMode(JOYSTICK_BUTTON, INPUT);
  tp.DotStar_SetPower( true );
  tp.DotStar_SetBrightness( 10 );
  tp.DotStar_CycleColor(25);
  // wait for serial port to open on native usb devices
  while (!Serial)
  {
    delay(1);
  }

  // initialize sensor
  encoder_init();
  Serial.println("Encoder done");

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

  timerWrite(timer0, 0);
  timerStart(timer0);
  timerWrite(timer1, 0);
  timerStart(timer1);

  tp.DotStar_SetPower( false ); 
  // tp.DotStar_SetBrightness( 20 );
  // delay(1000);
  // tp.DotStar_SetPixelColor( 255, 0, 0 );
  // delay(1000);
  // tp.DotStar_SetPixelColor( 0, 255, 0 );
  // delay(1000);
  // tp.DotStar_SetPixelColor( 0, 0, 255 );
  // delay(1000);
} 

void loop()
{
  switch (state)
  {
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
      wrist_MODE();
    }
    // if (digitalRead(CALIBRATION_BUTTON) == HIGH)
    // {
    //   // motor_STOP();
    //   // timerStop(timer0);
    //   // timerStop(timer1);
    //   calibrate_state = LOW;
    //   state = CALIBRATION;
    // }
    if (digitalRead(JOYSTICK_BUTTON) == HIGH)
    {
      state = JOYSTICK_MODE;
      // timerWrite(timer0, 0);
      // timerStart(timer0);
      // timerWrite(timer1, 0);
      // timerStart(timer1);
    }
    if (incoming == 'r')
    {
      encoder_count = 0;
    }

    // Serial.println('w');
    // delay(100);
    // tp.DotStar_SetPixelColor( 255, 128, 0);
    break;
  }

  // case CALIBRATION:
  // {
  //   calibrate();
  //   if (calibrate_state = HIGH)
  //   {
  //     state = WRIST_MODE;
  //   }
  //   // tp.DotStar_SetPower( false ); 
  //   break;
  // }

  case JOYSTICK_MODE:
  {
    byte incoming = Serial.read();
    tp.DotStar_SetPixelColor( 125, 125, 0 );

    if (timer1_check)
    {
      // Serial.println("timer 1");
      portENTER_CRITICAL(&timerMux1);
      timer1_check = false;
      portEXIT_CRITICAL(&timerMux1);
      joystick_MODE();
    }
    if (incoming == 'r')
    {
      encoder_count = 0;
    }
    joystick_MODE();
    //tp.DotStar_SetPower( false );
    //Serial.println('j');
    break;
  }

  }
}

// Setup functions --------------------------------------
void encoder_init()
{
  Serial.println("Motor encoder Initiation");

  ESP32Encoder::useInternalWeakPullResistors = puType::up; // Enable the weak pull up resistors
  encoder.attachHalfQuad(E2, E1);                 // Attache pins for use as encoder pins
  encoder.setCount(0);                             // set starting count value after attaching

  Serial.println("");
  delay(100);
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
   * float angle; // wrist angle from spectral flexible sensor
   */
  // Elapsed time
  elapsed_time = millis();

  // Wrist angle
  angle = analogRead(flexSensorPin);

  // Encoder count
  encoder_count += count;
  motor_speed = count;
  motor_acc = motor_speed - motor_speed_prev;
  motor_speed_prev = motor_speed;

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
  Serial.print(angle);
  Serial.print(", ");
  Serial.print(encoder_count);
  Serial.print(", ");
  Serial.print(motor_speed);
  Serial.print(", ");
  Serial.println(motor_acc);
}

// General functions -----------------------------------
void motor_STOP()
{
  ledcWrite(pwmChannel_1, LOW);
  ledcWrite(pwmChannel_2, LOW);
  //digitalWrite(LED_PIN, LOW);
}

void motor_FORWARD()
{
  ledcWrite(pwmChannel_1, JOYSTICK_PWM);
  ledcWrite(pwmChannel_2, LOW);
  //digitalWrite(LED_PIN, HIGH);
}

void motor_BACKWARD()
{
  ledcWrite(pwmChannel_1, LOW);
  ledcWrite(pwmChannel_2, JOYSTICK_PWM);
  //digitalWrite(LED_PIN, HIGH);
}

void calibrate()
{

  // Step 1. Set wrist angle to 0
  Serial.println("Set wrist angle to 0 degrees.");
  // led lights up to indicate user to commence calibration step
  tp.DotStar_SetPixelColor( 0, 255, 0 );
  waitForButtonPress();


  delay(10);

  //Step 2. Set wrist angle to 45
  Serial.println("Set wrist angle to 45 degrees.");
  //led lights up to indicate user to commence calibration step
  tp.DotStar_SetPixelColor( 0, 0, 255 );
  waitForButtonPress();

  delay(10);

  Serial.println("Calibration completed.");
  delay(10);
  calibrate_state = HIGH;
}

void waitForButtonPress()
{
  // Wait for the button press to proceed with calibration
  while (digitalRead(CALIBRATION_BUTTON) == LOW)
  {
    delay(50); // Poll the button state
  }

  delay(200); // Debounce delay

}

void joystick_MODE()
{
  delay(100);
  if (digitalRead(JOYSTICK_BUTTON) == true && digitalRead(CALIBRATION_BUTTON) == true)
  {
    motor_FORWARD();
  }
  else if (digitalRead(JOYSTICK_BUTTON) == true)
  {
    motor_BACKWARD();
  }
  else
  {
    motor_STOP();
  }
}

// Wrist angle control 3 for MEng team which doesn't include distance sensor(Nov.13.2023)
void wrist_MODE()
{
  switch (state2)
  {
  case IDLE:
  {
    tp.DotStar_SetPixelColor( 255, 0, 0 );
    motor_STOP();
    if (angle >= ON_ANGLE)
    {
      motor_FORWARD();
      state2 = CLOSING;
      Serial.println("closing");
    }
    else if (angle < OFF_ANGLE && encoder_count > 0)
    {
      motor_BACKWARD();
      state2 = OPENING;
      Serial.println("opening");
    }
    break;
  }
  case CLOSING:
  {
    tp.DotStar_SetPixelColor( 0, 255, 0 );
    if (motor_speed >= HIGH_VELOCITY)
    {
      motor_status = true;
    }

    if (motor_speed < LOW_VELOCITY && motor_status == true) 
    {
      motor_STOP();
      motor_status = false;
      state2 = GRASPING;
      Serial.println("grasping");
    }
    else if (angle < OFF_ANGLE)
    {
      motor_BACKWARD();
      motor_status = false;
      state2 = OPENING;
      Serial.println("opening");
    }
    // tp.DotStar_SetPixelColor( 255, 0, 0);
    break;
  }
  case OPENING:
  {
    tp.DotStar_SetPixelColor( 0, 0, 255 );
    if (encoder_count <= 0)
    {
      motor_STOP();
      state2 = IDLE;
      Serial.println("idle");
    }
    else if (angle >= ON_ANGLE)
    {
      motor_FORWARD();
      state2 = CLOSING;
      Serial.println("closing");
    }
    // tp.DotStar_SetPixelColor( 0, 255, 0);
    break;
  }
  case GRASPING:
  {
    if (angle < OFF_ANGLE)
    {
      motor_BACKWARD();
      state2 = OPENING;
      Serial.println("opening");
    }
    // tp.DotStar_SetPixelColor( 0, 0, 255);
    break;
  }

  default:
    break;
  }
}
