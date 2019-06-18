#include <ros.h>
#include <Wire.h>


ros::NodeHandle nh;

#include "ConfigTrimaran.h"
#include "ConfigIMU.h"


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
  Sensor();
  Actuator();
  
  delay(500);
}
