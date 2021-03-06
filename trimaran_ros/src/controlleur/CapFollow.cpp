#include <ros/ros.h>
#include <cstdlib>
#include <unistd.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose2D.h>
#include <math.h>
#include <gps_common/GPSFix.h>

using namespace std;


// x,y,cap
double x[3] = {0,0,0};
double xRef[3] = {0,0,0};
float wind = 0;
double yaw,pitch,roll;
double ax,ay,az;
double gx,gy,gz;
double timeImu;

float capCible =0;
float u[2];

void windCB(const std_msgs::Float32 msgWind){
    wind = msgWind.data;
    ROS_INFO("wind : %f",wind);
}

void magCB(const geometry_msgs::Vector3 msgMag)
{
    yaw = msgMag.x;
    pitch = msgMag.y;
    roll = msgMag.z;
    ROS_INFO("cap : %f",yaw);
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


void gpsCB(const gps_common::GPSFix msgGps)
{
    x[0] = 111.11*1000*(msgGps.latitude-xRef[0]);
    x[1] = -111.11*1000*(msgGps.longitude-xRef[1])*cos(xRef[0]*M_PI/180);
    x[2] = msgGps.track;
}

void capCB(const std_msgs::Float32 msgCap){
  if (msgCap.data != -999){
    capCible = msgCap.data+yaw;
  }
}



void capControl(){
    float deltar = atan(tan(0.5*(yaw-capCible)));
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

  int mode;
  nh.param<int>("mode", mode,0);
  nh.param<float>("cap",capCible,0);
  ROS_INFO("mode : %d", mode);

  string topicWind;
  string topicEuler;
  string topicImu;
  string topicGps;

  if (mode == 0){
    topicWind = "filter_send_wind_direction";
    topicEuler = "filter_send_euler_angles";
    topicImu = "ardu_send_imu";
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
  ros::Subscriber sub_capcible = nh.subscribe("camera_send_headings_arucos",0,capCB);

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
      //ROS_INFO("rudder : %f , Sail : %f",u[0],u[1]);
      pub_Rudder.publish(cmdrudder);
      pub_Sail.publish(cmdsail);
      loop_rate.sleep();
  }
    //sleep(1);
    return 0;
}
