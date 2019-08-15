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

// x,y,cap
int mode = 0;
double x[3] = {0,0,0};
double xRef[2] = {0,0};
double wind = 0;
double yaw,pitch,roll;
double ax,ay,az;
double gx,gy,gz;
double timeImu;
double q = 1;

vec2 a = {-10,0};
vec2 b = {10,0};

float u[2];

void refCB(const geometry_msgs::Vector3 msgRef){
  xRef[0] = msgRef.x;
  xRef[1] = msgRef.y;
}

void cubeACB(const geometry_msgs::Pose2D msgA){

    a[0] = 111.11*1000*(msgA.x-xRef[0]);
    a[1] = -111.11*1000*(msgA.y-xRef[1])*cos(xRef[0]*M_PI/180);

}

void cubeBCB(const geometry_msgs::Pose2D msgB){

    b[0] = 111.11*1000*(msgB.x-xRef[0]);
    b[1] = -111.11*1000*(msgB.y-xRef[1])*cos(xRef[0]*M_PI/180);

}

void windCB(const std_msgs::Float32 msgWind){
    wind = msgWind.data;
}

void magCB(const geometry_msgs::Vector3 msgMag)
{
    yaw = msgMag.x;
    pitch = msgMag.y;
    roll = msgMag.z;
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

void capControl(){
    double r = 15.0;
    double zeta = M_PI/4;


    vec2 m = {x[0],x[1]};

    double norm = glm::length(b-a);
    if (b==a){
      norm = 1;
    }
    mat2x2 mat;
    mat[0][0] = (b[0] - a[0]);
    mat[1][0] = (b[1] - a[1]);
    mat[0][1] = m[0] - a[0];
    mat[1][1] = m[1] - a[1];

    double e = glm::determinant(mat)/norm;
    double phi = atan2(b[1]-a[1],b[0]-a[0]);
    if (abs(e)>r*0.8){
        q = sign(e);
    }
    double thetabar = phi-atan(e/r);
    if ( ( cos(wind-thetabar) + cos(zeta) )<0){
        thetabar = M_PI+wind-zeta*q;
    }
    double deltar = (2/M_PI)*atan(tan(0.5*(yaw-thetabar)));
    double deltamax = (M_PI/4)*(cos(wind-thetabar)+1);

    u[0] = deltar;
    u[1] = deltamax;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "lineFollow");
  ros::NodeHandle nh;


  ros::Publisher  pub_Rudder = nh.advertise<std_msgs::Float32>("control_send_u_rudder",0);
  ros::Publisher  pub_Sail   = nh.advertise<std_msgs::Float32>("control_send_u_sail",0);

  ros::Subscriber sub_A = nh.subscribe("control_send_line_begin",0,cubeACB);
  ros::Subscriber sub_B = nh.subscribe("control_send_line_end",0,cubeBCB);
  ros::Subscriber sub_ref = nh.subscribe("control_send_ref",0,refCB);

  nh.param<int>("mode", mode,0);


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
      //ROS_INFO("x : %f , y : %f",x[0],x[1]);
      pub_Rudder.publish(cmdrudder);
      pub_Sail.publish(cmdsail);

      loop_rate.sleep();
  }
    //sleep(1);
    return 0;
}
