#include <SoftwareSerial.h>
#include <std_msgs/String.h>

SoftwareSerial SoftSerial(10, 11);
char buffer[200];
char position[200];
int count=0;   


std_msgs::String gpsMsg;
ros::Publisher pubGps("gpsBrut",&gpsMsg);
