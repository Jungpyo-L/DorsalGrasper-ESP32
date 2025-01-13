/*
    ESP-NOW Broadcast Master
    Lucas Saavedra Vaz - 2024

    This sketch demonstrates how to broadcast messages to all devices within the ESP-NOW network.
    This example is intended to be used with the ESP-NOW Broadcast Slave example.

    The master device will broadcast a message every 5 seconds to all devices within the network.
    This will be done using by registering a peer object with the broadcast address.

    The slave devices will receive the broadcasted messages and print them to the Serial Monitor.
*/
#include <esp_now.h>
// #include "ESP32_NOW.h"
#include "WiFi.h"

#include <esp_mac.h>  // For the MAC2STR and MACSTR macros

/* Definitions */



#define BUTTONa 12    // Green button
#define BUTTONb 15    // White button
#define BUTTONc 33    // Red button
#define LED 13
/* Classes */

// Creating a new class that inherits from the ESP_NOW_Peer class is required.
#define CHANNEL 1

// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

// config AP SSID
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

/* Global Variables */

uint32_t msg_count = 0;

// Create a broadcast peer object
// ESP_NOW_Broadcast_Peer broadcast_peer(ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, NULL);

/* Main */

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  pinMode(BUTTONa, INPUT);
  pinMode(BUTTONb, INPUT);
  pinMode(BUTTONc, INPUT);
  pinMode(LED, OUTPUT);
  // Initialize the Wi-Fi module
    Serial.println("ESPNow/Basic/Slave Example");
  //Set device in AP mode to begin with
    WiFi.mode(WIFI_AP);
    // configure device AP mode
    configDeviceAP();
    // This is the mac address of the Slave in AP Mode
    Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
    // Init ESPNow with a fallback logic
    InitESPNow();
    // Once ESPNow is successfully Init, we will register for recv CB to
    // get recv packer info.
    esp_now_register_recv_cb(OnDataRecv);
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Recv from: "); Serial.println(macStr);
  Serial.print("Last Packet Recv Data: "); Serial.println(*data);
  Serial.println("");
}

void loop() {
  // Broadcast a message to all devices within the network
  char data[32];
  Serial.printf("recieved message: %s\n", data);
  if (digitalRead(BUTTONa) == HIGH) {
    snprintf(data, sizeof(data), "green");
    digitalWrite(LED, HIGH);  // turn the LED on (HIGH is the voltage level)
    delay(1000);
    digitalWrite(LED, LOW);
  }
  if (digitalRead(BUTTONb) == HIGH) {
      //send to tinypico
      snprintf(data, sizeof(data), "white");
      digitalWrite(LED, HIGH);  // turn the LED on (HIGH is the voltage level)
      delay(1000);
      digitalWrite(LED, LOW);
  } 
  if (digitalRead(BUTTONc) == HIGH) {
      snprintf(data, sizeof(data), "red");
      digitalWrite(LED, HIGH);  // turn the LED on (HIGH is the voltage level)
      delay(1000);
      digitalWrite(LED, LOW);
  }

  // snprintf(data, sizeof(data), "Hello, World! #%lu", msg_count++);

  // Serial.printf("Broadcasting message: %s\n", data);

  // if (!broadcast_peer.send_message((uint8_t *)data, sizeof(data))) {
    // Serial.println("Failed to broadcast message");
    delay(1000);
  }

  // delay(10);

