
void setupRC(){
  pinMode(chPinRudder, INPUT);
  pinMode(chPinSail,INPUT);
}

void updateRC(){
 chSail = pulseIn (chPinSail,HIGH,duration);
 //Serial.println(ch1);
 if (chSail !=0){
   chRudder = pulseIn (chPinRudder,HIGH);
   chSail = pulseIn (chPinSail,HIGH);
   if (chRudder > 500){
      //Serial.print(chRudder);
      //Serial.print("  ");
      rudderAngle = 155*(chRudder-MIN_CHRUDDER)/(MAX_CHRUDDER-MIN_CHRUDDER);
      rudderAngle = max(minRudderAngle+maxRudderAngle/4,rudderAngle);
      rudderAngle = min(maxRudderAngle-maxRudderAngle/4,rudderAngle);
      watchRc = 1;
      Serial.print("rudder : ");
      Serial.print(chRudder);
      Serial.print(" ");
      Serial.println(rudderAngle);
   }
   if (chSail > 500){
      //Serial.print(ch3);
      //Serial.print("  ");
      sailAngle = maxSailAngle*(chSail-MIN_CHSAIL)/(MAX_CHSAIL-MIN_CHSAIL);
      sailAngle = max(minSailAngle,sailAngle);
      sailAngle = min(maxSailAngle,sailAngle);
      watchRc = 1;
      Serial.print("sail : ");
      Serial.print(chSail);
      Serial.print(" ");
      Serial.println(sailAngle);
  }
   
 }
 else{
  watchRc = 0;
 }
}
 
