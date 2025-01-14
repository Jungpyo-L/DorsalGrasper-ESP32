#include <dummy.h>


/*
 * author Jungpyo Lee: <jungpyolee@berkeley.edu> (c.)
 * creation date : May. 22. 2024
 * last update : sep. 23. 2024
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
#include <esp_now.h>
#include <WiFi.h>

#define CHANNEL 8
char macStr[18] = "0c:8b:95:96:5c:44";

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

char data[32];
const char* receivedData = reinterpret_cast<const char*>(data);

// char data[32];               // Buffer to store incoming data
// int lastSequence = -1;       // Variable to track the last processed sequence number
// bool newMessageReceived = false; // Flag to indicate a new message was received




//ESPNOW
// void InitESPNow() {
//   WiFi.disconnect();
//   if (esp_now_init() == ESP_OK) {
//     Serial.println("ESPNow Init Success");
//   }
//   else {
//     Serial.println("ESPNow Init Failed");
//     // Retry InitESPNow, add a counte and then restart?
//     // InitESPNow();
//     // or Simply Restart
//     ESP.restart();
//   }
// }
void InitESPNow() {
    WiFi.disconnect();
    int retries = 0;
    while (esp_now_init() != ESP_OK) {
        Serial.println("ESPNow Init Failed, retrying...");
        delay(100);  // Wait a bit before retrying
        retries++;
        if (retries > 3) {  // Limit retries to 3
            Serial.println("ESPNow Init Failed, please check hardware or configurations!");
            return;  // Consider a way to handle this in your main code
        }
    }
    Serial.println("ESPNow Init Success");
}

void configDeviceAP() {
  const char *SSID = "Slave_1";
  bool result = WiFi.softAP(SSID, "Slave_1_Password", CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
    Serial.print("AP CHANNEL "); Serial.println(WiFi.channel());
  }
}



// void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
//     char macStr[18];
//     snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
//              mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
//     Serial.print("Received from MAC: "); 
//     Serial.println(macStr);
//     Serial.print("Data: ");
//     Serial.write(data, data_len); // Make sure the data is displayed correctly
//     Serial.println();
//     Serial.println(receivedData);
// }

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int data_len) {          //original espnow message receiver
    // Ensure incoming data is null-terminated and does not exceed buffer size
    int safe_len = min(static_cast<int>(sizeof(data) - 1), data_len);
    memcpy(data, incomingData, safe_len);
    data[safe_len] = '\0'; // Ensure null termination

    // Since receivedData is already pointing to data, no need to redefine if it hasn't changed address
    // Optionally, you can explicitly set it here for clarity:
    receivedData = reinterpret_cast<const char*>(data);
    Serial.println(receivedData);

    // Now use receivedData as a C-string
    if (strcmp(receivedData, "red") == 0) {
        Serial.println("Red received");
    } else if (strcmp(receivedData, "green") == 0) {
        Serial.println("Green received");
    } else if (strcmp(receivedData, "white") == 0) {
        Serial.println("White received");
    }
}

// void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int data_len) {
//     int safe_len = min(static_cast<int>(sizeof(data) - 1), data_len);
//     memcpy(data, incomingData, safe_len);
//     data[safe_len] = '\0';

//     int receivedSequence;
//     sscanf(data, "%d", &receivedSequence);

//     Serial.print("Received data: ");
//     Serial.println(data);
//     Serial.print("Sequence received: ");
//     Serial.println(receivedSequence);

//     if (receivedSequence > lastSequence) {
//         lastSequence = receivedSequence;
//         newMessageReceived = true;
//         Serial.println("New message received. Flag set.");
//     } else {
//         Serial.println("Duplicate message ignored.");
//     }
// }





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

// const char* receivedData = reinterpret_cast<const char*>(data);


TinyPICO tp = TinyPICO();

void setup()
{
  Serial.begin(115200);
  // Serial.begin(921600);

  Serial.println("Start Setup");
  //ESPNOW
  WiFi.mode(WIFI_AP);
  // configure device AP mode
  configDeviceAP();
  // This is the mac address of the Slave in AP Mode
  // Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info.
  esp_now_register_recv_cb(OnDataRecv);
  // Serial.println(receivedData);
  
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


void loop() {
  static bool using_wrist_mode2 = true; // Variable to track the current mode
  
  // Check if the mode switch button is pressed
  // Assuming data is properly null-terminated and safe to use as a string
// const char* receivedData = reinterpret_cast<const char*>(data);
if (digitalRead(SWITCH_BUTTON) == HIGH || strcmp(data, "green") == 0) {
    // Serial.println("ESPNOW read green");
    using_wrist_mode2 = !using_wrist_mode2;
    Serial.println("Mode switch"); // Toggle between wrist_mode and wrist_mode2
    delay(100); // Debounce delay (consider a non-blocking alternative)
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
          // if (reinterpret_cast<const char*>(data) == "red"){
          //     Serial.print("ESPNOW read red\n");
          //   }
          Serial.print("button read red\n");
          // calibrate_state = LOW;
          state = JOYSTICK_MODE;
          delay(50);
        }
        //  if ( receivedData == "red"){
        //   Serial.print("ESPNOW read red\n");
        //   // calibrate_state = LOW;
        //   state = ESPNOW_MODE;
        //   delay(50);
        // }
        if (digitalRead(JOYSTICK_BUTTON) == HIGH || strcmp(data, "red") == 0 || strcmp(data, "white") == 0) {
          // if (reinterpret_cast<const char*>(data) == "white"){
          //     Serial.print("ESPNOW read white\n");
          //   }
          Serial.print("button read white\n");
          
          state = JOYSTICK_MODE;
          delay(50);
        } 
        // if ( receivedData == "white"){
        //   Serial.print("ESPNOW read white\n");
        //   state = ESPNOW_MODE;
        //   delay(50);
        // }
    //     if (receivedData) {                         // espnow
    //     if (strcmp(receivedData, "red") == 0) {
    //         // Handle red
    //         Serial.print("ESPNOW read red\n");
    //         // state = ESPNOW_MODE;
    //         state = JOYSTICK_MODE;
    //         delay(50);
    //     } else if (strcmp(receivedData, "green") == 0) {
    //         // Handle green
    //         Serial.print("ESPNOW read green\n");
    //     } else if (strcmp(receivedData, "white") == 0) {
    //         // Handle white
    //         Serial.print("ESPNOW read white\n");
    //         // state = ESPNOW_MODE;
    //         state = JOYSTICK_MODE;
    //         delay(50);
    //     }
    // }
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
  // case ESPNOW_MODE:     //espnow
  // {
  //    if (timer1_check)
  //   {
  //     // Serial.println("timer 1");
  //     portENTER_CRITICAL(&timerMux1);
  //     timer1_check = false;
  //     portEXIT_CRITICAL(&timerMux1);
  //     espnow_MODE();
  //   }
  //   espnow_MODE();
  //   break;
  // }

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
  //   Serial.print("forward, ");
  // } else if (state == JOYSTICK_MODE)
  // {
  //   Serial.print("backward, ");
  // } else if (state == SWITCH_MODE){
  //   Serial.print("mode switch, ")
  // }
  // Serial.print(elapsed_time);
  // Serial.print(", ");
  Serial.print(angle);
  Serial.print("\n, ");
  Serial.print(encoder_count);
  Serial.print(", ");
  Serial.print(motor_speed);
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


void joystick_MODE()           //original
{
  // delay(200);  //original 100
  if (digitalRead(CALIBRATION_BUTTON) == true || strcmp(data, "white") == 0)
  {
    motor_FORWARD();
    Serial.println("espnow: moving forward");
    delay(80);
    // tp.DotStar_SetPixelColor( 255, 0, 0 );
  }
  else if (digitalRead(JOYSTICK_BUTTON) == true || strcmp(data, "red") == 0)
  {
    motor_BACKWARD();
    Serial.println("espnow: moving backward");
    delay(80);
    // tp.DotStar_SetPixelColor( 0, 255, 0 );
  }
  else
  {
    motor_STOP();
  }
}

// void joystick_MODE() {
//     if (newMessageReceived) {
//         Serial.println("Processing new message...");
//         newMessageReceived = false; // Reset flag
        
//         if (strstr(data, "white") != NULL) {
//             motor_FORWARD();
//             Serial.println("espnow: moving forward");
//         } else if (strstr(data, "red") != NULL) {
//             motor_BACKWARD();
//             Serial.println("espnow: moving backward");
//         } else {
//             motor_STOP();
//             Serial.println("espnow: stopping motor");
//         }
//     } else {
//         Serial.println("No new message to process.");
//     }
// }



// void espnow_MODE(){               //espnow
//   if (strcmp(receivedData, "white") == 0)
//   {
//     motor_FORWARD();
//     Serial.println("espnow: moving forward");
//     state2 = CLOSING;
//     delay(80);
//     // motor_STOP();
//     // tp.DotStar_SetPixelColor( 255, 0, 0 );
//   }
//   else if (strcmp(receivedData, "red") == 0)
//   {
//     motor_BACKWARD();
//     Serial.println("espnow: moving backward");
//     state2 = OPENING;
//     delay(80);
//     // motor_STOP();
//     // tp.DotStar_SetPixelColor( 0, 255, 0 );
//   }
//   else
//   {
//     motor_STOP();
//   }
// }

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
