#include "HX711.h"

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 25;
const int LOADCELL_SCK_PIN = 26;

HX711 scale;

void setup() {
  Serial.begin(115200);
  scale.begin(A1, A0);
//  scale.set_scale(476.8);
  scale.tare();
}

void loop() {

//  if (scale.is_ready()) {
//    long reading = scale.read();
//    Serial.print("HX711 reading: ");
//    Serial.println(reading);
//  } else {
//    Serial.println("HX711 not found.");
//  }

Serial.println(scale.get_units(10));
  delay(1000);
  
}
