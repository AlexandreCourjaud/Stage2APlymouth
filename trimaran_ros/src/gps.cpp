#include <ros/ros.h>
#include <std_msgs/String.h>


void capCallbackCommande(const std_msgs::String msg)
{
    ROS_INFO(" %s ",msg.data.c_str());
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "gps");
  ros::NodeHandle nh;

  ros::Subscriber subCommande = nh.subscribe("gpsBrut", 1, capCallbackCommande);
  ros::Rate loop_rate(25);
  while (ros::ok()){
          ros::spinOnce();

          loop_rate.sleep();
  }
  sleep(1);
  return 0;
}
