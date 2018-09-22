int x = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("{GPS:");
  Serial.print(x);
  Serial.print(";AcX:12,33;}");
  x += 1;
  delay(200);
}
