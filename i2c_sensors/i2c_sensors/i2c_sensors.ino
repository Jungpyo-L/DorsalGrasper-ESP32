#include <Wire.h>
#include "Adafruit_VL6180X.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include "SparkFun_Displacement_Sensor_Arduino_Library.h"

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
#define INITIALIZATION 1 // Initialization state
#define JOYSTICK_MODE 2  // Joystick control mode
#define WRIST_MODE 3     // Wrist angle control mode

Adafruit_VL6180X vl = Adafruit_VL6180X();
Adafruit_MPU6050 mpu;
ADS ads;

uint32_t elapsed_time;
uint8_t distance;
sensors_event_t a, g, temp;
float angle;
int encoder_count;           // position of the motor
uint8_t j_L, j_R, pedal;     // joystick left, right, and pedal input

void setup() {
  pinMode(BTN_G, INPUT);
  pinMode(BTN_R, INPUT);
  pinMode(TC, INPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_R, OUTPUT);
  pinMode(JOYSTICK_L, INPUT);
  pinMode(JOYSTICK_R, INPUT);
  pinMode(FOOTPEDAL, INPUT);
  
  Serial.begin(115200);
  Wire.begin (23, 22);   // sda= GPIO_23 /scl= GPIO_22
  // wait for serial port to open on native usb devices

  // Sensor Initialization
  vl6080x_init();
  mpu6050_init();
  ads_init();
}

void loop() {
//  float lux = vl.readLux(VL6180X_ALS_GAIN_5);

//  Serial.print("Lux: "); Serial.println(lux);
  /*
  range = vl.readRange();
  uint8_t status = vl.readRangeStatus();

  Serial.print("Range: "); Serial.println(range);

  mpu.getEvent(&a, &g, &temp);
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");
  
  if (ads.available() == true)
  {
    angle = ads.getX();
    Serial.print("Wrist angle: ");
    Serial.print(angle);
    Serial.println();
  }
  */
  get_DATA();
  print_DATA();
  delay(50);
}

bool get_DATA() {

  elapsed_time = millis();
  
  distance = vl.readRange();

  if (ads.available())
  {
    angle = ads.getX();
  }

  mpu.getEvent(&a, &g, &temp);

  j_L = digitalRead(JOYSTICK_L);
  j_R = digitalRead(JOYSTICK_R);
  
}

bool print_DATA() {
  Serial.print(elapsed_time);
  Serial.print(", ");
  Serial.print(angle);
  Serial.print(", ");
  Serial.print(distance);
  Serial.print(", ");
  Serial.print(j_L);
  Serial.print(", ");
  Serial.print(j_R);
  Serial.print(", ");
  Serial.print(a.acceleration.x);
  Serial.print(", ");
  Serial.print(a.acceleration.y);
  Serial.print(", ");
  Serial.print(a.acceleration.z);
  Serial.print(", ");
  Serial.print(g.gyro.x);
  Serial.print(", ");
  Serial.print(g.gyro.y);
  Serial.print(", ");
  Serial.println(g.gyro.z);
}

bool vl6080x_init()
{
  Serial.println("Adafruit VL6180x test!");
  if (!vl.begin())
  {
    Serial.println("Failed to find sensor");
    while (1)
      ;
  }
  Serial.println("Sensor found!");
  Serial.println("");
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

bool ads_init()
{
  Serial.println("SparkFun Displacement Sensor Initiation");

  if (ads.begin() == false)
  {
    Serial.println(F("No sensor detected. Check wiring. Freezing..."));
    while (1)
      ;
  }
  delay(100);
}
