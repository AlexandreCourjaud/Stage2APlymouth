

void setupActuator(){
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  setupRudder();
  setupSail();
}



void Actuator(){
  controlRudder();
  controlSail();
}


/*************************************************************************************/

void setupSensor(){
  setupImu();
  //setupImuCmps();
  setupRC();
  setupWind();
  //setupGps();
}


void updateSensor(){
  //updateGps();
  updateImu();
  //updateImuCmps();
  updateRC();
  updateWind();
  
}
