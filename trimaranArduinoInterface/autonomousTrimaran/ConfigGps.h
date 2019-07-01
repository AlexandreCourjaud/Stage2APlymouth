#include <SoftwareSerial.h>
#include <gps_common/GPSFix.h>
#include "TinyGPS++.h"

//SoftwareSerial SoftSerial(10, 11);

char position[100];
int count=0;   
TinyGPSPlus gps;


gps_common::GPSFix gpsMsg;
ros::Publisher pubGps("ardu_send_gps",&gpsMsg);
