float ref = 0;

void setup() {
  Serial.begin(9600);
  pinMode(A1, INPUT);
}
void loop() {
  float sensorValue = analogRead(A1);
  Serial.print(sensorValue);
  float angle = ((sensorValue-ref-0)/(1023-0))*2*PI;
  angle = -2*atan(tan(angle/2));
  Serial.print("Angle : ");
  Serial.println(angle);
  delay(100);
}
