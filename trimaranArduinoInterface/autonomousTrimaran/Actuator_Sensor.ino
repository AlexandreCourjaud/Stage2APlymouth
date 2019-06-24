
//regroupe les setup et action des actionneurs 

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
  //setupGps();
  setupRC();
  setupWind();
}


void updateSensor(){
  updateImu();
  //updateGps();
  updateRC();
  updateWind();
}
