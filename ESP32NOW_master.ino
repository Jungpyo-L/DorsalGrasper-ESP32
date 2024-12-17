#include "ESP32_NOW.h"
#include "WiFi.h"

#include <esp_mac.h>  // For the MAC2STR and MACSTR macros

/* Definitions */

#define ESPNOW_WIFI_CHANNEL 8

#define BUTTONa 12    // Green button
#define BUTTONb 15    // White button
#define BUTTONc 33    // Red button
#define LED 13
/* Classes */
char data[32];

// Creating a new class that inherits from the ESP_NOW_Peer class is required.

class ESP_NOW_Broadcast_Peer : public ESP_NOW_Peer {
public:
  // Constructor of the class using the broadcast address
  ESP_NOW_Broadcast_Peer(uint8_t channel, wifi_interface_t iface, const uint8_t *lmk) : ESP_NOW_Peer(ESP_NOW.BROADCAST_ADDR, channel, iface, lmk) {}

  // Destructor of the class
  ~ESP_NOW_Broadcast_Peer() {
    remove();
  }

  // Function to properly initialize the ESP-NOW and register the broadcast peer
  bool begin() {
    if (!ESP_NOW.begin() || !add()) {
      log_e("Failed to initialize ESP-NOW or register the broadcast peer");
      return false;
    }
    return true;
  }

  // Function to send a message to all devices within the network
  bool send_message(const uint8_t *data, size_t len) {
    if (!send(data, len)) {
      log_e("Failed to broadcast message");
      return false;
    }
    return true;
  }
};

/* Global Variables */

// Create a broadcast peer object
ESP_NOW_Broadcast_Peer broadcast_peer(ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, NULL);

/* Main */

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(100);
  }
  pinMode(BUTTONa, INPUT);
  pinMode(BUTTONb, INPUT);
  pinMode(BUTTONc, INPUT);
  pinMode(LED, OUTPUT);
  // Initialize the Wi-Fi module
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) {
    delay(100);
  }

  Serial.println("ESP-NOW Example - Broadcast Master");
  Serial.println("Wi-Fi parameters:");
  Serial.println("  Mode: STA");
  Serial.println("  MAC Address: " + WiFi.macAddress());
  Serial.printf("  Channel: %d\n", ESPNOW_WIFI_CHANNEL);

  // Register the broadcast peer
  if (!broadcast_peer.begin()) {
    Serial.println("Failed to initialize broadcast peer");
    Serial.println("Rebooting in 5 seconds...");
    delay(5000);
    ESP.restart();
  }

  Serial.println("Setup complete. Waiting for button presses.");
}

void loop() {
  // Check button states and send messages accordingly
  if (digitalRead(BUTTONa) == HIGH) {
    snprintf(data, sizeof(data), "green");
    digitalWrite(LED, HIGH);  // Turn the LED on
    if (!broadcast_peer.send_message((uint8_t *)data, sizeof(data))) {
      Serial.println("Failed to broadcast green message");
    }
    delay(1000); // Debounce delay
    digitalWrite(LED, LOW);
  }

  if (digitalRead(BUTTONb) == HIGH) {
    snprintf(data, sizeof(data), "white");
    digitalWrite(LED, HIGH);  // Turn the LED on
    if (!broadcast_peer.send_message((uint8_t *)data, sizeof(data))) {
      Serial.println("Failed to broadcast white message");
    }
    delay(1000); // Debounce delay
    digitalWrite(LED, LOW);
  }

  if (digitalRead(BUTTONc) == HIGH) {
    snprintf(data, sizeof(data), "red");
    digitalWrite(LED, HIGH);  // Turn the LED on
    if (!broadcast_peer.send_message((uint8_t *)data, sizeof(data))) {
      Serial.println("Failed to broadcast red message");
    }
    delay(1000); // Debounce delay
    digitalWrite(LED, LOW);
  }

  delay(10); // Small delay to avoid rapid looping
}
