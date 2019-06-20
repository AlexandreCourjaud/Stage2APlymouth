float ref = 0;

void setup() {
  Serial.begin(9600);
  pinMode(A1, INPUT);
}
void loop() {
  float sensorValue = analogRead(A1);
  Serial.println(sensorValue);
  float angle = ((sensorValue-ref-45)/(975-40))*2*PI;
  angle = atan(tan(angle/2));
  Serial.println(angle);
  delay(100);
}
