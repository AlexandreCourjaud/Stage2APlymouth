

void setupRudder(){
  nh.subscribe(rudderSub);
  
}


void controlRudder(){
  pwm.setPWM(RUDDER_PIN,0,pulseWidth(rudderAngle));
  delay(10);
}

// commande between -pi/4 et pi/4
void rudderCallBack(const std_msgs::Float32& cmd){
  if (watchRc == 0){
    //if rc is off
    rudderAngle = min(PI/4,cmd.data);
    rudderAngle = max(-PI/4,rudderAngle);
    rudderAngle = minRudderAngle + (maxRudderAngle-minRudderAngle)*(rudderAngle+PI/4)/(PI/2);
    }
}
/*--------------------------------------------------------------*/


void setupSail(){
  nh.subscribe(sailSub);
}


void controlSail(){
  pwm.setPWM(SAIL_PIN,0,pulseWidth(sailAngle));
  delay(10);
}

//commande between 0 et pi/2
void sailCallBack(const std_msgs::Float32& cmd){
  if (watchRc == 0){
  //if rc is off
    sailAngle = max(0,cmd.data);
    sailAngle = min(PI/2,sailAngle);
    sailAngle = (sailAngle-0)*(maxSailAngle-minSailAngle)/(PI/2 - 0)+minSailAngle;
    
    
    //nh.loginfo("envoie angle sail...");
    }
}



int pulseWidth(int angle)
{
  int pulse_wide, analog_value;
  pulse_wide   = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  return analog_value;
}
