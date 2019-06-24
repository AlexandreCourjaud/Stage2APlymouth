#include <ros/ros.h>
#include <SoftwareSerial.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "gps");
  ros::NodeHandle nh;

  ROS_INFO("Hello world!");
}
