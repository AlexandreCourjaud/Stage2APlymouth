#include <SoftwareSerial.h>
#include <std_msgs/String.h>

SoftwareSerial SoftSerial(10, 11);
char buffer[64];
char position[64];
int count=0;   
int wait = 0;
int wait_max = 0;

std_msgs::String gpsMsg;
ros::Publisher pubGps("gpsBrut",&gpsMsg);
