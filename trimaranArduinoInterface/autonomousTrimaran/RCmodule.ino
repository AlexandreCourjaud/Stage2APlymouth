
void setupRC(){
  pinMode(chA, INPUT);
  pinMode(chB,INPUT);
}

void updateRC(){
 ch1 = pulseIn (chA,HIGH,duration);
 //Serial.println(ch1);
 if (ch1 !=0){
   ch1 = pulseIn (chA,HIGH);
   ch3 = pulseIn (chB,HIGH);
   if (ch1 > 500){
      //Serial.print(ch1);
      //Serial.print("  ");
      rudderAngle = 155*(ch1-MIN_CH1)/(MAX_CH1-MIN_CH1);
      rudderAngle = max(minRudderAngle+maxRudderAngle/4,rudderAngle);
      rudderAngle = min(maxRudderAngle-maxRudderAngle/4,rudderAngle);
      watchRc = 1;
      //Serial.println(rudderAngle);
   }
   if (ch3 > 500){
      //Serial.print(ch3);
      //Serial.print("  ");
      sailAngle = maxSailAngle*(ch3-MIN_CH3)/(MAX_CH3-MIN_CH3);
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
 
