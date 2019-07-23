#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define MIN_PULSE_WIDTH       650
#define MAX_PULSE_WIDTH       2350
#define DEFAULT_PULSE_WIDTH   1500
#define FREQUENCY             50






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
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  Serial.begin(115200);
}

void loop() {
  
    
if (Serial.available()) {
    int inByte = Serial.read();
    //Serial.write(inByte);
    apply(inByte);
  }
pwm.setPWM(1, 0, pulseWidth(50));
delay(10);
}

void servo(){
  
  pwm.setPWM(1, 0, pulseWidth(120));
  pwm.setPWM(12, 0, pulseWidth(10));
  delay(2000);
  Serial.println("go");
  pwm.setPWM(1, 0, pulseWidth(0));
  pwm.setPWM(12, 0, pulseWidth(140));

  delay(500);
}

int pulseWidth(int angle)
{
  int pulse_wide, analog_value;
  pulse_wide   = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  return analog_value;
}
