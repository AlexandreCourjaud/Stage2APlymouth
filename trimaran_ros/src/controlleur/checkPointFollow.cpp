#include <ros/ros.h>
#include <cstdlib>
#include <unistd.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose2D.h>
#include <math.h>
#include <geometry_msgs/Point.h>
#include <glm/glm.hpp>
#include <iostream>
#include <fstream>

#include <fcntl.h>



using namespace std;
using namespace glm;

double x[3] = {0,0,0};
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

void gpsCB(const geometry_msgs::Pose2D msgGps)
{
    x[0] = msgGps.x;
    x[1] = msgGps.y;
    x[2] = msgGps.theta;
}

/*
void control(){
  char *filename = (char*)"/checkpoint/test.txt";
  int file;
  if ((file = open(filename, O_RDWR)) < 0)
  {
          //ERROR HANDLING: you can check errno to see what went wrong
          ROS_INFO("Failed to open");
  }
}
*/




int main(int argc, char **argv)
{
  ros::init(argc, argv, "controlLine");
  ros::NodeHandle nh;

  ros::Publisher  pub_A = nh.advertise<geometry_msgs::Point>("control_send_A",0);
  ros::Publisher  pub_B = nh.advertise<geometry_msgs::Point>("control_send_B",0);

  geometry_msgs::Point msgA;
  geometry_msgs::Point msgB;

  //ros::Subscriber sub_Wind = nh.subscribe("filter_send_wind_direction",0,windCB);
  //ros::Subscriber sub_Mag  = nh.subscribe("filter_send_euler_angles",0,magCB);
  //ros::Subscriber sub_Imu  = nh.subscribe("filter_send_imu",0,imuCB);
  //ros::Subscriber sub_gps  = nh.subscribe("filter_send_gps",0,gpsCB);

  ros::Subscriber sub_Wind = nh.subscribe("simu_send_wind",0,windCB);
  ros::Subscriber sub_Mag  = nh.subscribe("simu_send_euler_angle",0,magCB);
  ros::Subscriber sub_Imu  = nh.subscribe("simu_send_imuSensor",0,imuCB);
  ros::Subscriber sub_gps  = nh.subscribe("simu_send_gps",0,gpsCB);


  ifstream myfile("package://trimaran_ros/include/test.txt");
  ros::Rate loop_rate(25);
  while (ros::ok()){
      ros::spinOnce();


      //control();
      msgA.x = a[0];
      msgA.y = a[1];

      msgB.x = b[0];
      msgB.y = b[1];

      pub_A.publish(msgA);
      loop_rate.sleep();
  }
    //sleep(1);
    return 0;

}
