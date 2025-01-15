#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h> // For the MAC2STR and MACSTR macros

/* Definitions */
#define BUTTONa 12    // Green button
#define BUTTONb 15    // White button
#define BUTTONc 33    // Red button
#define LED 13

/* Global Variables */
char data[32];
uint8_t targetMAC[] = {0x0C, 0x8B, 0x95, 0x96, 0x5C, 0x44}; // Target MAC address

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(100);
  }

  pinMode(BUTTONa, INPUT);
  pinMode(BUTTONb, INPUT);
  pinMode(BUTTONc, INPUT);
  pinMode(LED, OUTPUT);

  // Initialize Wi-Fi in STA mode
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(); // Ensure no active connection
  while (!WiFi.STA.started()) {
    delay(100);
  }

  // Print device MAC and channel information
  Serial.println("ESP-NOW Example - Peer-to-Peer");
  Serial.println("Wi-Fi parameters:");
  Serial.println("  Mode: STA");
  Serial.println("  MAC Address: " + WiFi.macAddress());

  // Initialize ESP-NOW
  if (!ESP_NOW.begin()) {
    Serial.println("Failed to initialize ESP-NOW");
    while (true) {
      delay(1000);
    }
  }

  // Register the peer with the target MAC address
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, targetMAC, 6);
  peerInfo.channel = 0; // Use the same Wi-Fi channel
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("Setup complete. Waiting for button presses.");
}

void loop() {
  // Check button states and send messages accordingly
  if (digitalRead(BUTTONa) == HIGH) {
    snprintf(data, sizeof(data), "green");
    Serial.println("Sending green message");
    digitalWrite(LED, HIGH);  // Turn the LED on
    if (esp_now_send(targetMAC, (uint8_t *)data, sizeof(data)) != ESP_OK) {
      Serial.println("Failed to send green message");
    }
    delay(1000); // Debounce delay
    digitalWrite(LED, LOW);
  } else if (digitalRead(BUTTONb) == HIGH) {
    snprintf(data, sizeof(data), "white");
    Serial.println("Sending white message");
    digitalWrite(LED, HIGH);  // Turn the LED on
    if (esp_now_send(targetMAC, (uint8_t *)data, sizeof(data)) != ESP_OK) {
      Serial.println("Failed to send white message");
    }
    delay(1000); // Debounce delay
    digitalWrite(LED, LOW);
  } else if (digitalRead(BUTTONc) == HIGH) {
    snprintf(data, sizeof(data), "red");
    Serial.println("Sending red message");
    digitalWrite(LED, HIGH);  // Turn the LED on
    if (esp_now_send(targetMAC, (uint8_t *)data, sizeof(data)) != ESP_OK) {
      Serial.println("Failed to send red message");
    }
    delay(1000); // Debounce delay
    digitalWrite(LED, LOW);
  } else {
    delay(10); // Do nothing
  }

  delay(10); // Small delay to avoid rapid looping
}
