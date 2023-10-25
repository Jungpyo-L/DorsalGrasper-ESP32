float a, b;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  a = random(100);
  b = random(100);
  Serial.print(a/100, 4);
  Serial.print(",");
  Serial.println(b/100, 4);
  delay(10);
}
