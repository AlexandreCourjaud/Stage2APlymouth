 #include <ros.h>
#include <Wire.h>


ros::NodeHandle nh;

// Include the right configuration for the motor and rc 

#include "ConfigMonohull.h"
//#include "ConfigCatamaran.h"
//#include "ConfigTrimaran.h"

// configuration for the sensors

#include "ConfigIMU.h"
#include "ConfigImuCmps12.h"
#include "ConfigGps.h"
#include "ConfigWind.h"


bool watchRc = 0;
bool gp = 0;

// init the ros node, actuator and sensor
void setup()
{
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  Serial.begin(115200);
  Wire.begin();
  //Serial.println("Initializing I2C devices...");
  setupActuator();
  setupSensor();
 
  
  
}

// lood update sensor and publish it, then update actuator according to publisher
void loop()
{
  nh.spinOnce();
  //nh.loginfo("next");
  
  
  updateSensor();
  Actuator();
  
  //delay(1);
}
