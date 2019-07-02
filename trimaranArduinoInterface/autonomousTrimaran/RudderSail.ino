

void setupRudder(){
  nh.subscribe(rudderSub);
}


void controlRudder(){
  pwm.setPWM(RUDDER_PIN,0,pulseWidth(rudderAngle));
  delay(10);
}

// commande entre -pi/4 et pi/4
void rudderCallBack(const std_msgs::Float32& cmd){
  if (watchRc == 0){
    //si la telecommande n'est pas allumee
    rudderAngle = 155*(cmd.data+PI/2)/PI;
    rudderAngle = max(minRudderAngle+maxRudderAngle/4,rudderAngle);
    rudderAngle = min(maxRudderAngle*3/4,rudderAngle);
    //nh.loginfo("envoie angle ...");
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

//commande entre 0 et pi/2
void sailCallBack(const std_msgs::Float32& cmd){
  if (watchRc == 0){
  //si la telecommande n'est pas allumee
    sailAngle = maxSailAngle*cmd.data/(PI/2);
    sailAngle = max(minSailAngle,sailAngle);
    sailAngle = min(maxSailAngle,sailAngle);
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
