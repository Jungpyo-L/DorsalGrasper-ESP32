
/*
  ESP32 MAC Address printout
  esp32-mac-address.ino
  Prints MAC Address to Serial Monitor
 
  DroneBot Workshop 2022
  https://dronebotworkshop.com
*/
 
// Include WiFi Library
#include "WiFi.h"
 
void setup() {
 
  // Setup Serial Monitor
  Serial.begin(115200);
 
  // Put ESP32 into Station mode
  WiFi.mode(WIFI_MODE_STA);
 
  // Print MAC Address to Serial monitor
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
}
 
void loop() {
 Serial.println(WiFi.macAddress());
 delay(5000);
}