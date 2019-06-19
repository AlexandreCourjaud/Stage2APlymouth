
#include <Servo.h>

Servo myRudder;  // create servo object to control a servo
Servo mySail;


void nothing(){
  Serial.println("nothing\n");
}



void apply(int b){
  switch (b){
    case 111: // "o"
      servo();
      break;
    default:
      nothing();
  
  }
}



void setup() {
  myRudder.attach(4);
  mySail.attach(5);
  Serial.begin(115200);
}

void loop() { 
  if (Serial.available()) {
      int inByte = Serial.read();
      //Serial.write(inByte);
      apply(inByte);
    }
}

void servo(){

  mySail.write(120);
  myRudder.write(10);
  delay(2000);
  Serial.println("go");
  mySail.write(0);
  myRudder.write(140);
  delay(500);
}
