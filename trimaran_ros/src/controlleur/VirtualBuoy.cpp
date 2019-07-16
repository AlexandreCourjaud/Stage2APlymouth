#include <ros/ros.h>
#include <cstdlib>
#include <unistd.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose2D.h>
#include <math.h>
#include <glm/glm.hpp>
#include <geometry_msgs/Point.h>
#include <gps_common/GPSFix.h>

using namespace std;
using namespace glm;

double x[3] = {0,0,0};
double xRef[2] = {0,0};
float wind = 0;
double yaw,pitch,roll;
double ax,ay,az;
double gx,gy,gz;
double timeImu;

vec2 a = {0,5};
vec2 b = {30,30};



void windCB(const std_msgs::Float32 msgWind){
    wind = msgWind.data;
    //ROS_INFO("wind : %f",wind);
}

void magCB(const geometry_msgs::Vector3 msgMag)
{
    yaw = msgMag.x;
    pitch = msgMag.y;
    roll = msgMag.z;
    //ROS_INFO("cap : %f",yaw);
}

void imuCB(const sensor_msgs::Imu msgImu)
{
    ax = msgImu.linear_acceleration.x;
    ay = msgImu.linear_acceleration.y;
    az = msgImu.linear_acceleration.z;

    gx = msgImu.angular_velocity.x;
    gy = msgImu.angular_velocity.y;
    gz = msgImu.angular_velocity.z;

    timeImu = msgImu.header.stamp.nsec;
}




void gpsCB(const gps_common::GPSFix msgGps){
  x[0] = msgGps.latitude;
  x[1] = msgGps.longitude;
  x[2] = msgGps.track;
}



void control(){
    a[0] = x[0];
    a[1] = x[1];
}





int main(int argc, char **argv)
{
  ros::init(argc, argv, "controlLine");
  ros::NodeHandle nh;

  ros::Publisher  pub_A = nh.advertise<geometry_msgs::Point>("control_send_A",0);
  ros::Publisher  pub_B = nh.advertise<geometry_msgs::Point>("control_send_B",0);
  ros::Publisher  pub_Ref = nh.advertise<geometry_msgs::Point>("control_send_ref",0);

  geometry_msgs::Point msgA;
  geometry_msgs::Point msgB;
  geometry_msgs::Point msgRef;

  msgRef.x = xRef[0];
  msgRef.y = xRef[1];

  int mode;
  nh.param<int>("mode", mode,0);
  ROS_INFO("mode : %d", mode);

  string topicWind;
  string topicEuler;
  string topicImu;
  string topicGps;

  if (mode == 0){
    topicWind = "filter_send_wind_direction";
    topicEuler = "filter_send_euler_angles";
    topicImu = "filter_send_imu";
    topicGps = "filter_send_gps";
  }
  else{
    topicWind = "simu_send_wind_direction";
    topicEuler = "simu_send_euler_angles";
    topicImu = "simu_send_imu";
    topicGps = "simu_send_gps";
  }
  ros::Subscriber sub_Wind = nh.subscribe(topicWind,0,windCB);
  ros::Subscriber sub_Mag  = nh.subscribe(topicEuler,0,magCB);
  ros::Subscriber sub_Imu  = nh.subscribe(topicImu,0,imuCB);
  ros::Subscriber sub_gps  = nh.subscribe(topicGps,0,gpsCB);
  ros::Rate loop_rate(25);
  while (ros::ok()){
      ros::spinOnce();


      control();
      msgA.x = a[0];
      msgA.y = a[1];
      pub_A.publish(msgA);

      msgB.x = xRef[0];
      msgB.y = xRef[1];
      pub_B.publish(msgB);

      pub_Ref.publish(msgRef);
      loop_rate.sleep();
  }
    //sleep(1);
    return 0;

}
