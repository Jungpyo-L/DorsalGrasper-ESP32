#include <ESP32Encoder.h>

ESP32Encoder encoder;

void setup(){
	Serial.begin(115200);
	ESP32Encoder::useInternalWeakPullResistors=UP;  // Enable the weak pull up resistors
	encoder.attachHalfQuad(27, 33); // Attache pins for use as encoder pins
	encoder.setCount(0);  // set starting count value after attaching
}

void loop(){
	Serial.println("Encoder count = "+String((int32_t)encoder.getCount())); // Loop and read the count
	delay(100);
}
