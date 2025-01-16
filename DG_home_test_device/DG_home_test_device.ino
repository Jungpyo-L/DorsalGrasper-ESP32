/*
 * author Jungpyo Lee: <jungpyolee@berkeley.edu> (c.) and Alahe Akhavan
 * creation date : Sep. 15. 2024
 * last update : Jan. 15. 2025
 * version 2.0
 * changed: add ESPNOW mode
 * project : Home based evaluation of the Dorsal Grasper (Main device)
 * Note: Use ESP32 board version to 3.x.x, which makes many changes
 * update list: https://docs.espressif.com/projects/arduino-esp32/en/latest/migration_guides/2.x_to_3.0.html
 */

// Include headers --------------------------------------
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <SparkFun_Displacement_Sensor_Arduino_Library.h>
#include <ESP32Encoder.h>
// #include <TinyPICO.h>
#include <esp_now.h>
#include <WiFi.h>


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
#define ESPNOW_MODE 9

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
const int MAX_ANGLE = 1000; // maximum angle of the wrist, 900
const int MIN_ANGLE = 550; // minimum angle of the wrist, 450
const int ON_ANGLE =825; // on angle to close the finger
const int OFF_ANGLE =680; // off angle to open the inger
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


// ESPNOW variables -------------------------------------
typedef struct DG_message {
  char mode;
  bool a;
  bool b;
  bool c;
  char joystick;
} DG_message;

// Create a struct_message called myData
DG_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  // Elapsed time
  static unsigned long lastTime = 0;
  unsigned long now = millis();
  Serial.print("Callback at ");
  Serial.print(now);
  Serial.print(" ms. Interval: ");
  Serial.println(now - lastTime);
  lastTime = now;
  // Wrist angle
  angle = analogRead(flexSensorPin);
  // Encoder count
  count = -encoder.getCount();
  encoder.clearCount();
  encoder_count += count;
  motor_speed = count;
  motor_acc = motor_speed - motor_speed_prev;
  motor_speed_prev = motor_speed;

  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Mode: ");
  Serial.println(myData.mode);
  Serial.print("Button a: ");
  Serial.println(myData.a);
  Serial.print("Button b: ");
  Serial.println(myData.b);
  Serial.print("Button c: ");
  Serial.println(myData.c);
  Serial.print("Joystick left: ");
  Serial.println(myData.joystick);
  Serial.print("Wrist angle: ");
  Serial.println(angle);
  Serial.print("Encoder count: ");
  Serial.println(encoder_count);
  Serial.print("Motor speed: ");
  Serial.println(motor_speed);
  Serial.println();
  
}

void setup()
{
  Serial.begin(115200);
  // Serial.begin(921600);

  Serial.println("Start Setup");
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Once ESPNow is successfully Init, this will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  
  pinMode(CALIBRATION_BUTTON, INPUT);
  pinMode(JOYSTICK_BUTTON, INPUT);
  pinMode(SWITCH_BUTTON, INPUT);    //new switch
  

  // wait for serial port to open on native usb devices
  while (!Serial)
  {
    delay(1);
  }

  // initialize sensor
  encoder_init();

  // initialize motor
  Serial.println("Motor PWM Initiation");

  // configure LED PWM functionalitites
  ledcAttach(MOTOR_A, freq, resolution);
  ledcAttach(MOTOR_B, freq, resolution);
  delay(100);

  // initialize timer; timer0 = data acquisition, timer1 = motor operation
  timer0 = timerBegin(1000);             // timer 0
  timerAttachInterrupt(timer0, &onTime0);
  timerAlarm(timer0, 50, true, 0); 
  timerStart(timer0);

  timer1 = timerBegin(1000);             // timer 1
  timerAttachInterrupt(timer1, &onTime1);
  timerAlarm(timer1, 50, true, 0); 
  timerStart(timer1);

  Serial.println("");
  Serial.println("Setup done");
} 


void loop() {

  switch (myData.mode) {
    case 'i':
      {
        break;
      }

    case 'j':
      {
        joystick_MODE();
        break;
      }
    case 'w':
      {
        wrist_MODE_disc();
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


// General functions -----------------------------------
void motor_STOP()
{
  ledcWrite(MOTOR_A, LOW);
  ledcWrite(MOTOR_B, LOW);
}

void motor_FORWARD()
{
  ledcWrite(MOTOR_A, JOYSTICK_PWM);
  ledcWrite(MOTOR_B, LOW);
}

void motor_BACKWARD()
{
  ledcWrite(MOTOR_A, LOW);
  ledcWrite(MOTOR_B, JOYSTICK_PWM);
}

void calibrate()
{

  // Step 1. Set wrist angle to 0
  Serial.println("Set wrist angle to 0 degrees.");
  // led lights up to indicate user to commence calibration step
  waitForButtonPress();


  delay(10);

  //Step 2. Set wrist angle to 45
  Serial.println("Set wrist angle to 45 degrees.");
  //led lights up to indicate user to commence calibration step
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


void joystick_MODE()           //original
{
  if (myData.joystick == 'l')
  {
    motor_FORWARD();
    // Serial.println("espnow: moving forward");
  }
  else if (myData.joystick == 'r')
  {
    motor_BACKWARD();
    // Serial.println("espnow: moving backward");
  }
  else
  {
    motor_STOP();
  }
}


// Discritized wrist mode
void wrist_MODE_disc()
{
  switch (state2)
  {
  case IDLE:
  {
    // tp.DotStar_SetPixelColor( 255, 0, 0 );
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
    // tp.DotStar_SetPixelColor( 0, 255, 0 );
    if (motor_speed >= HIGH_VELOCITY)
    // if (motor_speed >= LOW_VELOCITY)
    {
      motor_status = true;
    }

    // if (motor_speed < LOW_VELOCITY && motor_status == true) 
    // if (motor_speed < HIGH_VELOCITY && motor_status == true) 

    if (motor_speed < LOW_VELOCITY && motor_status == true)
    // if (encoder_count>=1500 && motor_status == true)  
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
    // tp.DotStar_SetPixelColor( 0, 0, 255 );
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

// continouseous wrist mode
void wrist_MODE_cont()
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
    ledcWrite(MOTOR_A, duty);
    ledcWrite(MOTOR_B, LOW);
  }
  else if (duty < 0)
  {
    ledcWrite(MOTOR_A, LOW);
    ledcWrite(MOTOR_B, -duty);
  }
  else if (duty == 0)
  {
    ledcWrite(MOTOR_A, LOW);
    ledcWrite(MOTOR_B, LOW);
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
