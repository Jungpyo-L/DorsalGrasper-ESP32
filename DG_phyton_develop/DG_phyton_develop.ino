/*
 * author Jungpyo Lee: <jungpyolee@berkeley.edu> (c.)
 * creation date : Feb. 17. 2022
 * last update : Feb. 17. 2022
 * version 1.0
 * project : Dorsal Grasper v1.1
 * motivation : to build data acquisition GUI with python
 */

// Define states ----------------------------------------
#define CALIBRATION 0    // Calibration state
#define INITIALIZATION 1 // Initialization state
#define JOYSTICK_MODE 2  // Joystick control mode
#define WRIST_MODE 3     // Wrist angle control mode

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

// Record variables -------------------------------------
int state = INITIALIZATION;
float ax, ay, az, gx, gy, gz;
uint8_t distance, vl_status; // vl6080x
unsigned long elapsed_time, t1, t2, t3, t4, t5;  // elapsed time
float temperature;           // temperature from ad8405
int encoder_count;           // position of the motor
uint8_t j_L, j_R, pedal;     // joystick left, right, and pedal input
float angle;                 // wrist angle from ads

// Setup variables --------------------------------------
const int freq = 20000;
const int pwmChannel_1 = 1;
const int pwmChannel_2 = 2;
const int resolution = 8; // PWM value from 0 to 255)
const int MAX_PWM_VOLTAGE = 255;
const int NOM_PWM_VOLTAGE = 150;
const int JOYSTICK_PWM = 200;
bool calibrate_state;

void setup()
{
  Serial.begin(115200);
  Serial.println("Start Setup");
  pinMode(BTN_G, INPUT);
  pinMode(BTN_R, INPUT);

  // wait for serial port to open on native usb devices
  while (!Serial)
  {
    delay(1);
  }
  
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_R, LOW);
  calibrate_state = LOW;
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
    
    if (incoming == 'c')
    {
      digitalWrite(LED_G, HIGH);
      digitalWrite(LED_R, HIGH);
      state = CALIBRATION;
      calibrate_state = LOW;
      delay(10);
      break;
    }
    if (incoming == 'j')
    {
      digitalWrite(LED_R, LOW);
      digitalWrite(LED_G, HIGH);
      state = JOYSTICK_MODE;
      delay(10);
      break;
    }
    if (incoming == 'w')
    {
      digitalWrite(LED_R, HIGH);
      digitalWrite(LED_G, LOW);
      state = WRIST_MODE;
      delay(10);
      break;
    }
    print_DATA();
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
    if (button_BOTH())
    {
      digitalWrite(LED_G, HIGH);
      digitalWrite(LED_R, HIGH);
      state = INITIALIZATION;
      delay(10);
    }
    
    get_DATA();
    print_DATA();
    break;
  }

  case WRIST_MODE:
  {
    if (button_BOTH())
    {
      digitalWrite(LED_G, HIGH);
      digitalWrite(LED_R, HIGH);
      state = INITIALIZATION;
      delay(10);
    }
    get_DATA();
    print_DATA();
    break;
  }
  }
  delay(100);
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
  // Elapsed time
  elapsed_time = millis();

  // Wrist angle
  angle = random(0, 60);

  // Distance (device to object)
  distance = random(0, 200);

  // Motor encoder count
  encoder_count = random(0, 1000);

  // Joystick input
  j_L = 0;
  j_R = 0;

  // Pedal input
  pedal = 1;

  // Accecleration and Gyro
  ax = 1 + random(-1, 1);
  ay = -0.1 + random(-1, 1);
  az = 9.8 + random(-1, 1);
  gx = 3 + random(-1, 1);
  gy = -1 + random(-1, 1);
  gz = 0.8 + random(-1, 1);
  
  // Motor temperature
  temperature = random(20, 40);

}

void print_DATA()
{
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
  Serial.print(j_L);w
  Serial.print(", ");
  Serial.print(j_R);
  Serial.print(", ");
  Serial.print(pedal);
  Serial.print(", ");
  Serial.print(ax);
  Serial.print(", ");
  Serial.print(ay);
  Serial.print(", ");
  Serial.print(az);
  Serial.print(", ");
  Serial.print(gx);
  Serial.print(", ");
  Serial.print(gy);
  Serial.print(", ");
  Serial.print(gz);
  Serial.print(", ");
  Serial.println(temperature);
}

// General functions -----------------------------------
void calibrate()
{
  Serial.println("c");

  while (Serial.available() > 0)
   Serial.read(); //Flush all characters
  //Serial.println(F("Press a key when the wrist is 0 degree angle"));
  while (Serial.available() == 0)
  {
//    ads.available();
    delay(10); //Wait for user to press character
  }

// ads.calibrateZero(); //Call when sensor is straight on both axis

  Serial.println("0");

  while (Serial.available() > 0)
    Serial.read(); //Flush all characters
  //Serial.println(F("Good. Now press a key when the wrist is bent at 45 degrees (extension)."));
  while (Serial.available() == 0)
  {
//    ads.available();
    delay(10); //Wait for user to press character
  }

//  ads.calibrateX45(); //Call when sensor is 45 degrees on X axis

  Serial.println("45");

  calibrate_state = HIGH;
}
