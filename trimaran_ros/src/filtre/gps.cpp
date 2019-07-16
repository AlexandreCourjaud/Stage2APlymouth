#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3.h>


double x[3] = {0,0,0};
double xRef[2] = {0,0};
int newVal = 0;

void gpsCB(const sensor_msgs::NavSatFix msgGps){
  x[0] = 111.11*1000*(msgGps.latitude-xRef[0]);
  x[1] = 111.11*1000*(msgGps.longitude-xRef[1])*cos(xRef[0]*M_PI/180);
  newVal = 1;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "gps");
  ros::NodeHandle nh;



  ros::Subscriber sub_Gps = nh.subscribe("fix",0,gpsCB);
  ros::Publisher pub_Gps_cart = nh.advertise<geometry_msgs::Vector3>("filter_send_gps",10);
  geometry_msgs::Vector3 positionMsg;


  ros::Rate loop_rate(25);
  while (ros::ok()){
      ros::spinOnce();
      if (newVal == 1){
        positionMsg.x = x[0];
        positionMsg.y = x[1];
        pub_Gps_cart.publish(positionMsg);
      }

      loop_rate.sleep();
  }
    return 0;
}
