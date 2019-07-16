
const byte pinAnemo = 2;
int compteur =0;

void anemoInterrupt(){
  compteur++;
  Serial.println(compteur);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(pinAnemo,INPUT);
  attachInterrupt(digitalPinToInterrupt(pinAnemo), anemoInterrupt,FALLING);
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println(digitalRead(pinAnemo));
}
