#include <ros/ros.h>
#include <cstdlib>
#include <unistd.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose2D.h>
#include <math.h>

using namespace std;


// x,y,cap
int x[3] = {0,0,0};
float wind = 0;
double yaw,pitch,roll;
double ax,ay,az;
double gx,gy,gz;
double timeImu;

float capCible = 0;
float u[2];

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


void capControl(){
    float deltar = -0.5*atan(tan(0.5*(yaw-capCible)));
    float deltamax = (M_PI/4)*(cos(wind-capCible)+1);


    u[0] = deltar;
    u[1] = deltamax;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "CapFollow");
  ros::NodeHandle nh;


  ros::Publisher  pub_Rudder = nh.advertise<std_msgs::Float32>("control_send_u_rudder",0);
  ros::Publisher  pub_Sail   = nh.advertise<std_msgs::Float32>("control_send_u_sail",0);

  //ros::Subscriber sub_Wind = nh.subscribe("filter_send_wind_direction",0,windCB);
  //ros::Subscriber sub_Mag  = nh.subscribe("filter_send_euler_angles",0,magCB);
  //ros::Subscriber sub_Imu  = nh.subscribe("imuSensor",0,imuCB);
  //ros::Subscriber sub_gps  = nh.subscribe("gpsPos",0,gpsCB);

  ros::Subscriber sub_Wind = nh.subscribe("simu_send_wind",0,windCB);
  ros::Subscriber sub_Mag  = nh.subscribe("simu_send_angleEuler",0,magCB);
  ros::Subscriber sub_Imu  = nh.subscribe("simu_send_imuSensor",0,imuCB);
  ros::Subscriber sub_gps  = nh.subscribe("simu_send_gpsPos",0,gpsCB);
  u[0] = 0;
  u[1] = 0;

  std_msgs::Float32 cmdrudder;
  std_msgs::Float32 cmdsail;

  ros::Rate loop_rate(25);
  while (ros::ok()){
      ros::spinOnce();

      capControl();
      cmdrudder.data = u[0];
      cmdsail.data = u[1];
      ROS_INFO("rudder : %f , Sail : %f",u[0],u[1]);
      pub_Rudder.publish(cmdrudder);
      pub_Sail.publish(cmdsail);

      loop_rate.sleep();
  }
    //sleep(1);
    return 0;
}
