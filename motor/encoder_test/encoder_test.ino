#define ENC_1 26
#define ENC_2 25

void setup() {
  // put your setup code here, to run once:
  pinMode(ENC_1,INPUT);
  pinMode(ENC_2,INPUT);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("E1 = " + String(digitalRead(ENC_1)) + ", E2 = " + String(digitalRead(ENC_2)));
}
