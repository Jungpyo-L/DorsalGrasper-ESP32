/*
  Reading the one and two axis flex sensors from Bend Labs
  By: Nathan Seidle @ SparkFun Electronics
  Date: March 2nd, 2019
  License: This code is public domain but you buy me a beer if you use this 
  and we meet someday (Beerware license).

  This example reads the flex values of the single or 
  dual axis angular bend sensor (ADS)

  SparkFun labored with love to create this code. Feel like supporting open
  source? Buy a sensor from SparkFun!
  https://www.sparkfun.com/products/15245 (2-axis sensor)
  https://www.sparkfun.com/products/15244 (1-axis sensor)

  Hardware Connections:
  Use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  to connect to the RedBoard Qwiic and the following pins on the ADS:
  SCL: Yellow wire on Qwiic cable
  SDA: Blue
  VCC: Red
  GND: Black

  Single axis pinout: https://cdn.sparkfun.com/assets/9/f/8/2/d/Bendlabs_Single_Axis_Flex_Sensor_Pinout.png
  Dual axis pintout: https://cdn.sparkfun.com/assets/f/f/9/e/6/Bendlabs_Dual_Axis_Flex_Sensor_Pinout.png
  
  Open the serial monitor at 115200 baud to see the output
*/

#include <Wire.h>
#include "SparkFun_Displacement_Sensor_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_Displacement_Sensor

ADS myFlexSensor; //Create instance of the Angular Displacement Sensor (ADS) class

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("SparkFun Displacement Sensor Example");

  Wire.begin(23, 22);

  if (myFlexSensor.begin() == false)
  {
    Serial.println(F("No sensor detected. Check wiring. Freezing..."));
    while (1)
      ;
  }
  
  // Serial.println("clear calibration");
  // myFlexSensor.clearCalibration();
  // delay(500);
  // Serial.println("clear done");
  // myFlexSensor.calibrateZero();

}

void loop()
{
//  myFlexSensor.clearCalibration();
// Serial.println("Hi");
  if (myFlexSensor.available() == true)
  {
    Serial.print(myFlexSensor.getX());
    Serial.println();
  }

  delay(10);
}
