#define TC 32
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Temperature = ");
  Serial.print(get_temperature());
  Serial.println(" C");
  delay(500);
}

// AD8495 (Thermal courple) functions
float get_temperature() {
  float AREF = 3.3;
  int ADC_RESOLUTION = 12;
  float reading, voltage, temperature;
  
  reading = analogRead(TC);
  voltage = reading * (AREF / (pow(2,ADC_RESOLUTION)-1));
  temperature = (voltage - 1.25) / 0.005;

  return temperature;
}
