#include <ros/ros.h>
#include <cstdlib>
#include <unistd.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose2D.h>
#include <math.h>
#include <glm/glm.hpp>


using namespace std;
using namespace glm;

// x,y,cap
int x[3] = {0,0,0};
float wind = 0;
double yaw,pitch,roll;
double ax,ay,az;
double gx,gy,gz;
double timeImu;

vec2 a = {0,5};
vec2 b = {30,30};

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
    float r = 5.0;
    float zeta = M_PI/4;
    float q = 1;

    vec2 m = {x[0],x[1]};
    float norm = glm::length(m);
    mat2x2 mat;
    mat[0][0] = (b[0] - a[0]) / norm;
    mat[1][0] = (b[1] - b[1]) / norm;
    mat[0][1] = m[0] - a[0];
    mat[1][1] = m[1] - a[1];

    float e = glm::determinant(mat);

    float phi = atan2(b[1]-a[1],b[0]-a[0]);

    if (abs(e)>r){
        q = sign(e);
    }
    float thetabar = phi-atan(e/r);
    if ((cos(wind-thetabar)+cos(zeta))<0){
        thetabar = M_PI+wind-zeta*q;
    }

    float deltar = -0.5*atan(tan(0.5*(yaw-thetabar)));
    float deltamax = (M_PI/4)*(cos(wind-thetabar)+1);


    u[0] = deltar;
    u[1] = deltamax;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "lineFollow");
  ros::NodeHandle nh;


  ros::Publisher  pub_Rudder = nh.advertise<std_msgs::Float32>("control_send_u_rudder",0);
  ros::Publisher  pub_Sail   = nh.advertise<std_msgs::Float32>("control_send_u_sail",0);

  //ros::Subscriber sub_Wind = nh.subscribe("filter_send_wind_direction",0,windCB);
  //ros::Subscriber sub_Mag  = nh.subscribe("filter_send_euler_angles",0,magCB);
  //ros::Subscriber sub_Imu  = nh.subscribe("filter_send_imu",0,imuCB);
  //ros::Subscriber sub_gps  = nh.subscribe("filter_send_gps",0,gpsCB);

  ros::Subscriber sub_Wind = nh.subscribe("Simu_send_wind",0,windCB);
  ros::Subscriber sub_Mag  = nh.subscribe("Simu_send_angleEuler",0,magCB);
  ros::Subscriber sub_Imu  = nh.subscribe("Simu_send_imuSensor",0,imuCB);
  ros::Subscriber sub_gps  = nh.subscribe("Simu_send_gpsPos",0,gpsCB);
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
