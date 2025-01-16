/*
 * author Jungpyo Lee: <jungpyolee@berkeley.edu> (c.)
 * creation date : Jan. 15. 2025
 * last update : Jan. 15. 2025
 * version 1.0
 * project : Home based evaluation of the Dorsal Grasper (remote controller with ESPnow)
 * source: https://randomnerdtutorials.com/esp-now-esp32-arduino-ide/
 */


#include <WiFi.h>
#include <esp_now.h>

/* Definitions */
#define BUTTONa 12    // Green button
#define BUTTONb 11    // White button
#define BUTTONc 10    // Red button
#define LED 13
#define VRX_PIN  9 // ESP32-S3 pin connected to VRX pin
#define VRY_PIN  6 // ESP32-S3 pin connected to VRY pin
#define SW_PIN   5  // ESP32-S3 pin connected to SW  pin

#define LEFT_THRESHOLD  400
#define RIGHT_THRESHOLD 3695

// Setup joystick variables
int xValue = 0; // To store value of the X axis
int yValue = 0; // To store value of the Y axis
int bValue = 0; // To store value of the button


// update these with values suitable for your network.
// test TinyPICO {0x64, 0xB7, 0x08, 0x91, 0x9E, 0x84}
// Main device {0xD4, 0xD4, 0xDA, 0xAA, 0x36, 0x38}
uint8_t broadcastAddress[] = {0xD4, 0xD4, 0xDA, 0xAA, 0x36, 0x38};


// Setup interrupt variables --------------------------------
volatile bool timer0_check = false; // check timer interrupt 0
hw_timer_t *timer0 = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;


// Setup interrupt functions ----------------------------
void IRAM_ATTR onTime0()
{ // this can be used for constant data acquisition
  portENTER_CRITICAL_ISR(&timerMux0);
  timer0_check = true; // the function to be called when timer interrupt is triggered
  portEXIT_CRITICAL_ISR(&timerMux0);
}


// Structure for the Dorsal Grasper to send data
// Must match the receiver structure
typedef struct DG_message {
  char mode;
  bool a;
  bool b;
  bool c;
  char joystick;
} DG_message;

// Create a struct_message called myData
DG_message myData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  pinMode(BUTTONa, INPUT);
  pinMode(BUTTONb, INPUT);
  pinMode(BUTTONc, INPUT);
  pinMode(LED, OUTPUT);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // new version
  timer0 = timerBegin(1000);             // Frequency-based API (1000 = 1000 Hz -> 1 ms per tick)
  timerAttachInterrupt(timer0, &onTime0);            // Attach your ISR
  timerAlarm(timer0, 50, true, 0);                   // Fire every 50 ticks (50 ms), autoreload, 0 reload count
  timerStart(timer0);                                // Start the timer
}



void loop() {
  static char mode = 'i';

  if (timer0_check)
  {
    portENTER_CRITICAL(&timerMux0);
    timer0_check = false;
    portEXIT_CRITICAL(&timerMux0);
    get_DATA();
    if (myData.a) {
      mode = 'i';
    }
    else if (myData.b) {
      mode = 'j';
    }
    else if (myData.c) {
      mode = 'w';
    }
    else {
      digitalWrite(LED, LOW);
    }
    myData.mode = mode;
    send_DATA();
    reset_DATA();
  }
}

void get_DATA() {
  myData.a = digitalRead(BUTTONa);
  myData.b = digitalRead(BUTTONb);
  myData.c = digitalRead(BUTTONc);
  xValue = analogRead(VRX_PIN);
  yValue = analogRead(VRY_PIN);
  // check left/right commands
  if (xValue < LEFT_THRESHOLD){
    myData.joystick = 'l';
  }
  else if (xValue > RIGHT_THRESHOLD){
    myData.joystick = 'r';
  }
  else{
    myData.joystick = 'i';
  }
}

void send_DATA() {
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
}

void reset_DATA() {
  myData.a = false;
  myData.b = false;
  myData.c = false;
}