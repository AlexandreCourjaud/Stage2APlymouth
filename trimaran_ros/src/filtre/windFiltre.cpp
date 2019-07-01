#include <ros/ros.h>
#include <std_msgs/Float32.h>

float wind;

void windCB(const std_msgs::Float32 msgWind){
    wind = msgWind.data;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "windFiltre");
  ros::NodeHandle nh;

  ROS_INFO("Hello world!");
}
