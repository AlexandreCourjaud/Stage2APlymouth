
void setupRC(){
  pinMode(chPinRudder, INPUT);
  pinMode(chPinSail,INPUT);
}

void updateRC(){
 chRudder = pulseIn (chPinRudder,HIGH,duration);
 //Serial.println(ch1);
 if (chRudder !=0){
   chRudder = pulseIn (chPinRudder,HIGH);
   chSail = pulseIn (chPinSail,HIGH);
   if (chRudder > 500){
      //Serial.print(chRudder);
      //Serial.print("  ");
      rudderAngle = 155*(chRudder-MIN_CHRUDDER)/(MAX_CHRUDDER-MIN_CHRUDDER);
      rudderAngle = max(minRudderAngle+maxRudderAngle/4,rudderAngle);
      rudderAngle = min(maxRudderAngle-maxRudderAngle/4,rudderAngle);
      watchRc = 1;
      //Serial.println(rudderAngle);
   }
   if (chSail > 500){
      //Serial.print(ch3);
      //Serial.print("  ");
      sailAngle = maxSailAngle*(chSail-MIN_CHSAIL)/(MAX_CHSAIL-MIN_CHSAIL);
      sailAngle = max(minSailAngle,sailAngle);
      sailAngle = min(maxSailAngle,sailAngle);
      watchRc = 1;
      //Serial.println(sailAngle);
  }
   
 }
 else{
  watchRc = 0;
 }
}
 
