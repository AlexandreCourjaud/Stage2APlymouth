 #include <ros.h>
#include <Wire.h>


ros::NodeHandle nh;

#include "ConfigMonohull.h"
//#include "ConfigTrimaran.h"
#include "ConfigIMU.h"
#include "ConfigImuCmps12.h"
#include "ConfigGps.h"
//#include "ConfigRC.h"
#include "ConfigWind.h"


bool watchRc = 0;
bool gp = 0;

void setup()
{
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  Serial.begin(115200);
  Wire.begin();
  Serial.println("Initializing I2C devices...");
  setupActuator();
  setupSensor();
 
  
  
}


// la loop declenche verifie les publisher, lit les capteurs et actionne les servo
void loop()
{
  nh.spinOnce();
  //nh.loginfo("next");
  
  Actuator();
  updateSensor();
  
  delay(1);
}
