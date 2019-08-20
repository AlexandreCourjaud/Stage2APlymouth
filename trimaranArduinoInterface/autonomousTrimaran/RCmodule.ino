
void setupRC(){
  pinMode(chPinRudder, INPUT);
  pinMode(chPinSail,INPUT);
  nh.advertise(pubRc);
  timerRc = 0;
}

void updateRC(){
 // check if rc is On
 chRudder = pulseIn (chPinRudder,HIGH,duration);

 
 //Serial.println(ch1);
 if (chRudder !=0){
   chRudder = pulseIn (chPinRudder,HIGH);
   chSail = pulseIn (chPinSail,HIGH);
   timerRc = millis();
   if (chRudder > 500){
      //Serial.print(chRudder);
      //Serial.print("  ");
      rudderAngle = minRudderAngle + (maxRudderAngle-minRudderAngle)*(chRudder-MIN_CHRUDDER)/(MAX_CHRUDDER-MIN_CHRUDDER);
      rudderAngle = max( min(minRudderAngle,maxRudderAngle),rudderAngle);
      rudderAngle = min(max(minRudderAngle,maxRudderAngle),rudderAngle);
      watchRc = 1;
      /*
      Serial.print("rudder : ");
      Serial.print(chRudder);
      Serial.print(" ");
      Serial.println(rudderAngle);
      */
      
   }
   if (chSail > 500){
      //Serial.print(ch3);
      //Serial.print("  ");
      sailAngle = minSailAngle + (maxSailAngle-minSailAngle)*(chSail-MIN_CHSAIL)/(MAX_CHSAIL-MIN_CHSAIL);
      sailAngle = max(min(minSailAngle,maxSailAngle),sailAngle);
      sailAngle = min(max(maxSailAngle,minSailAngle),sailAngle);
      watchRc = 1;
      /*
      Serial.print("sail : ");
      Serial.print(chSail);
      Serial.print(" ");
      Serial.println(sailAngle);
      */
      
  }
   
 }
 rcMsg.x = rudderAngle;
 rcMsg.y = sailAngle;
 rcMsg.z = watchRc;
 pubRc.publish(&rcMsg);

 // if rc off for 1 second : switch in autonomous mode 
 if (millis()-timerRc > 1000){
    watchRc = 0;
 }
}
 
