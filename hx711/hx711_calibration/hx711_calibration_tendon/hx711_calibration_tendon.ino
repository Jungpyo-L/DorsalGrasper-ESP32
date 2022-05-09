#include "HX711.h"

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
#define LOADCELL_DOUT_PIN 15    // Loadcell 1 data
#define LOADCELL_SCK_PIN 32    // Loadcell 1 clock
#define TC 14       // Thermal couple analog input
#define LOADCELL_DOUT_PIN_2 22      // Loadcell 2 data
#define LOADCELL_SCK_PIN_2 23      // Loadcell 2 clock

HX711 scale2;

void setup() {
  Serial.begin(115200);
  scale2.begin(LOADCELL_DOUT_PIN_2, LOADCELL_SCK_PIN_2);
  scale2.set_scale(239);
  scale2.tare();
}

void loop() {

//  if (scale.is_ready()) {
//    long reading = scale.read();
//    Serial.print("HX711 reading: ");
//    Serial.println(reading);
//  } else {
//    Serial.println("HX711 not found.");
//  }

Serial.println(scale2.get_units(10));
  delay(1000);
  
}
