#include <dummy.h>


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
#define flexSensorPin  26 // flexible sensor pin

#define SWITCH_BUTTON 25    //new switch

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
const int JOYSTICK_PWM = 250; // motor PWM value for the joystick mode
const int WRIST_PWM = 220; // motor PWM value for the wrist angle mode
const int MAX_EN = 1600; // encoder value in fully closed finger
const int MAX_ANGLE = 900; // maximum angle of the wrist, 900
const int MIN_ANGLE = 450; // minimum angle of the wrist, 450
const int ON_ANGLE = 550; // on angle to close the finger
const int OFF_ANGLE =450; // off angle to open the inger
const int HIGH_VELOCITY = 50; // High threshold velocity
const int LOW_VELOCITY = 30; // Low threshold velocity, grasp force
bool calibrate_state;
const int Kp = 2;      // P gain
const int Ki = 0.1;     // I gain
const int Kd = 0;       // D gain


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
  // pinMode(CALIBRATION_BUTTON, INPUT_PULLUP);
  // pinMode(JOYSTICK_BUTTON, INPUT_PULLUP);
  
  // attachInterrupt(digitalPinToInterrupt(CALIBRATION_BUTTON), isrCalibration, FALLING);
  // attachInterrupt(digitalPinToInterrupt(JOYSTICK_BUTTON), isrJoystick, FALLING);
  pinMode(SWITCH_BUTTON, INPUT);    //new switch
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
  // Serial.println("Encoder done");

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
  // Serial.println("Timer0 initializatoin");
  timer0 = timerBegin(0, 80, true);             // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer0, &onTime0, true); // edge (not level) triggered
  timerAlarmWrite(timer0, 50000, true);        // 50000 * 1 us = 50 ms, autoreload true (20 Hz)
  timerAlarmEnable(timer0);                     // enable timer0
  timerStop(timer0);

  // Serial.println("Timer1 initializatoin");
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

// void loop()
// {
//   switch (state)
//   {
//   case WRIST_MODE:
//   {
//     byte incoming = Serial.read();
//     if (timer0_check)
//     {
//       // Serial.println("timer 0");
//       portENTER_CRITICAL(&timerMux0);
//       timer0_check = false;
//       portEXIT_CRITICAL(&timerMux0);
//       get_DATA();
//       print_DATA();
//     }
//     if (timer1_check)
//     {
//       // Serial.println("timer 1");
//       portENTER_CRITICAL(&timerMux1);
//       timer1_check = false;
//       portEXIT_CRITICAL(&timerMux1);
//       // wrist_MODE(); // on/off wrist angle control mode
//       wrist_MODE2(); // continous wrist angle control mode
//     }
//     if (digitalRead(CALIBRATION_BUTTON) == HIGH)  //calibration
//     {
//       // motor_STOP();
//       // timerStop(timer0);
//       // timerStop(timer1);
//       calibrate_state = LOW;
//       // state = CALIBRATION;
//       state = JOYSTICK_MODE;
//     }
//     if (digitalRead(JOYSTICK_BUTTON) == HIGH)
//     {
//       state = JOYSTICK_MODE;
//       // timerWrite(timer0, 0);
//       // timerStart(timer0);
//       // timerWrite(timer1, 0);
//       // timerStart(timer1);
//     }
//     // if (incoming == 'r')
//     // {
//     //   encoder_count = 0;
//     // }

//     // Serial.println('w');
//     // delay(100);
//     // tp.DotStar_SetPixelColor( 255, 128, 0);
//     break;
//   }

void loop() {
  static bool using_wrist_mode2 = true; // Variable to track the current mode
  
  // Check if the mode switch button is pressed
  if (digitalRead(SWITCH_BUTTON) == HIGH) {
    using_wrist_mode2 = !using_wrist_mode2; // Toggle between wrist_mode and wrist_mode2
    delay(200); // Debounce delay
  }
  if (using_wrist_mode2) {
            wrist_MODE2(); // Continuous wrist angle control mode
            tp.DotStar_SetPixelColor( 125,125, 0 );
          } else {
            wrist_MODE(); // On/Off wrist angle control mode
            tp.DotStar_SetPixelColor( 125,125, 0 );
          }

  switch (state) {
    case WRIST_MODE:
      {
        byte incoming = Serial.read();
        if (timer0_check) {
          portENTER_CRITICAL(&timerMux0);
          timer0_check = false;
          portEXIT_CRITICAL(&timerMux0);
          get_DATA();
          print_DATA();
        }
        if (timer1_check) {
          portENTER_CRITICAL(&timerMux1);
          timer1_check = false;
          portEXIT_CRITICAL(&timerMux1);
          if (using_wrist_mode2) {
            wrist_MODE2(); // Continuous wrist angle control mode
          } else {
            wrist_MODE(); // On/Off wrist angle control mode
          }
          
        }
        if (digitalRead(CALIBRATION_BUTTON) == HIGH) {
          // calibrate_state = LOW;
          state = JOYSTICK_MODE;
          delay(50);
        }
        if (digitalRead(JOYSTICK_BUTTON) == HIGH) {
          state = JOYSTICK_MODE;
          delay(50);
        }
        break;
      }

    // Existing cases for CALIBRATION and JOYSTICK_MODE...

//   }
// }


  case CALIBRATION:   //calibration
  {
    calibrate();
    if (calibrate_state = HIGH)
    {
      state = WRIST_MODE;
    }
    // tp.DotStar_SetPower( false ); 
    break;
  }

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
    // if (incoming == 'r')
    // {
    //   encoder_count = 0;
    // }
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
  // Serial.println("Motor encoder Initiation");

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
  // if (state == WRIST_MODE)
  // {
  //   // Serial.print("w, ");
  // } else if (state == JOYSTICK_MODE)
  // {
  //   Serial.print("j, ");
  // }
  // Serial.print(elapsed_time);
  // Serial.print(", ");
  // Serial.print(angle);
  // Serial.print(", ");
  // Serial.print(encoder_count);
  // Serial.print(", ");
  // Serial.print(motor_speed);
  // Serial.print(", ");
  // Serial.println(motor_acc);
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
// void isrCalibration() {
//   state = CALIBRATION;
// }

// void isrJoystick() {
//   state = JOYSTICK_MODE;
// }
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

// void joystick_MODE()
// {
//   static unsigned long lastDebounceTime = 50;
//   const unsigned long debounceDelay = 80;  // 80ms debounce time
  
//   // Get current time
//   unsigned long currentTime = millis();
  
//   if (digitalRead(CALIBRATION_BUTTON) == HIGH)
//   {
//     if (currentTime - lastDebounceTime > debounceDelay)
//     {
//       motor_FORWARD();
//       lastDebounceTime = currentTime;
//       // tp.DotStar_SetPixelColor(255, 0, 0);
//     }
//   }
//   else if (digitalRead(JOYSTICK_BUTTON) == HIGH)
//   {
//     if (currentTime - lastDebounceTime > debounceDelay)
//     {
//       motor_BACKWARD();
//       lastDebounceTime = currentTime;
//       // tp.DotStar_SetPixelColor(0, 255, 0);
//     }
//   }
//   else
//   {
//     motor_STOP();
//   }
// }

void joystick_MODE()
{
  // delay(200);  //original 100
  if (digitalRead(CALIBRATION_BUTTON) == true)
  {
    motor_FORWARD();
    delay(80);
    // tp.DotStar_SetPixelColor( 255, 0, 0 );
  }
  else if (digitalRead(JOYSTICK_BUTTON) == true)
  {
    motor_BACKWARD();
    delay(80);
    // tp.DotStar_SetPixelColor( 0, 255, 0 );
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
    // if (angle <= ON_ANGLE && encoder_count <=900)
    {
      motor_FORWARD();
      state2 = CLOSING;
      Serial.println("closing");
    } 
    // else if (angle < OFF_ANGLE)   
    else if (angle < OFF_ANGLE && encoder_count >= 0)
    {
      motor_BACKWARD();
      state2 = OPENING;
      Serial.println("opening");
      // motor_STOP();
    }
    break;
  }
  case CLOSING:
  {
    tp.DotStar_SetPixelColor( 0, 255, 0 );
    if (motor_speed >= HIGH_VELOCITY)
    // if (motor_speed >= LOW_VELOCITY)
    {
      motor_status = true;
    }

    // if (motor_speed < LOW_VELOCITY && motor_status == true) 
    // if (motor_speed < HIGH_VELOCITY && motor_status == true) 

    // if (motor_speed < LOW_VELOCITY && motor_status == true)
    if (encoder_count>=1500 && motor_status == true)  
    {
      motor_STOP();
      motor_status = false;
      state2 = GRASPING;
      Serial.println("grasping");
    }
    // else if (angle < ON_ANGLE)
    else if (angle < OFF_ANGLE)
    {
      motor_BACKWARD();
      motor_status = false;
      state2 = OPENING;
      Serial.println("opening");
      // motor_STOP();
    }
    // tp.DotStar_SetPixelColor( 255, 0, 0);
    break;
  }
  case OPENING:
  {
    tp.DotStar_SetPixelColor( 0, 0, 255 );
    if (encoder_count <= 0)     //encoder
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
    // if (angle < ON_ANGLE && angle > OFF_ANGLE)
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


void wrist_MODE2()
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
  }
  else if (duty < 0)
  {
    ledcWrite(pwmChannel_1, LOW);
    ledcWrite(pwmChannel_2, -duty);
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
