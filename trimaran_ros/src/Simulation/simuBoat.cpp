#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <glm/glm.hpp>
#include <vector>
#include <visualization_msgs/Marker.h>
#include "tf/tf.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <gps_common/GPSFix.h>


using namespace std;
using namespace glm;

double xRef[2] = {0,0};

double wind,awind;
double cmdRudder, cmdSail;

double delta_s;
double p[10] = {0.1,1,6000,1000,2000,1,1,2,300,10000};

//x,y,cap,v,w: vitesse rotation
double x[5] = {0,0,0,0,0};
double xdot[5] = {0,0,0,0,0};


double t0;

vec2 cubeA = {-10,0};
vec2 cubeB = {10,0};


void windCB(const std_msgs::Float32 msgWind){
    wind = msgWind.data;
}

void rudderCB(const std_msgs::Float32 msgRudder){
    cmdRudder = msgRudder.data;
}

void sailCB(const std_msgs::Float32 msgSail){
    cmdSail = msgSail.data;
}

void refCB(const geometry_msgs::Point msgRef){
  xRef[0] = msgRef.x;
  xRef[1] = msgRef.y;
}

/***********************************************************************************************/

void f(){
    double theta = x[2];
    double v = x[3];
    double w = x[4];
    vec2 w_ap;
    w_ap[0] = awind*cos(wind-theta)-v;
    w_ap[1] = awind*sin(wind-theta);
    double psi_ap = atan2(w_ap[1],w_ap[0]);
    double a_ap = glm::length(w_ap);
    double sigma = cos(psi_ap)+cos(cmdSail);
    if (sigma<0){
        delta_s = M_PI + psi_ap;
    }
    else{
        delta_s = -sign(sin(psi_ap))*cmdSail;
    }
    double fr = p[4]*v*sin(cmdRudder);
    double fs = p[3]*a_ap*sin(delta_s-psi_ap);
    xdot[0] = v*cos(theta) + p[0]*awind*cos(wind);
    xdot[1] = v*sin(theta) + p[0]*awind*sin(wind);
    xdot[2] = w;
    xdot[3] = (fs*sin(delta_s) - fr*sin(cmdRudder) - p[1]*v*v)/p[8];
    xdot[4] = (fs*(p[5]-p[6]*cos(delta_s))- p[7]*fr*cos(cmdRudder)-p[2]*w*v)/p[9];

}


void euler(){
    double t1 = ros::Time::now().toSec();
    double dt = t1 - t0;
    //double dt = 0.1;
    x[0] = x[0] + dt*xdot[0];
    x[1] = x[1] + dt*xdot[1];
    x[2] = x[2] + dt*xdot[2];
    x[3] = x[3] + dt*xdot[3];
    x[4] = x[4] + dt*xdot[4];
    t0 = t1;
}

/********************************************************************************************************************/

void set_imu(ros::Publisher pub_imu, sensor_msgs::Imu msgImu, double x[5]){
    msgImu.linear_acceleration.x = 0;
    msgImu.linear_acceleration.y = 0;
    msgImu.linear_acceleration.z = 0;

    msgImu.angular_velocity.x  = 0;
    msgImu.angular_velocity.y  = 0;
    msgImu.angular_velocity.z  = 0;
    pub_imu.publish(msgImu);
}

void set_gps(ros::Publisher pub_gps, gps_common::GPSFix msgGps, double x[5]){
  msgGps.latitude = x[0]/(111.11*1000)+ xRef[0];
  msgGps.longitude = -x[1]/(111.11*1000*cos(xRef[0]*M_PI/180))+xRef[1];
  msgGps.track = x[2];
  msgGps.speed = x[3];
  pub_gps.publish(msgGps);
}


void set_Euler(ros::Publisher pub_Euler, geometry_msgs::Vector3 msgEuler, double x[5]){
    msgEuler.x = x[2];
    msgEuler.y = 0;
    msgEuler.z = 0;
    pub_Euler.publish(msgEuler);
}

void set_wind(ros::Publisher pub_wind, std_msgs::Float32 msgWind){
    msgWind.data = wind;
    pub_wind.publish(msgWind);
}

void set_awind(ros::Publisher pub_awind, std_msgs::Float32 msgaWind){
    msgaWind.data = awind;
    pub_awind.publish(msgaWind);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "simuBoat");
    ros::NodeHandle nh;

    ros::Subscriber sub_sail = nh.subscribe("control_send_u_sail",0,sailCB);
    ros::Subscriber sub_rudder = nh.subscribe("control_send_u_rudder",0,rudderCB);
    ros::Subscriber sub_wind = nh.subscribe("filter_send_wind_direction",0,windCB);
    ros::Subscriber sub_ref = nh.subscribe("control_send_ref",0,refCB);

    ros::Publisher  pub_imu = nh.advertise<sensor_msgs::Imu>("simu_send_imu",0);
    sensor_msgs::Imu msgImu;
    ros::Publisher pubGps = nh.advertise<gps_common::GPSFix>("simu_send_gps",0);
    gps_common::GPSFix msgGps;
    ros::Publisher pub_wind = nh.advertise<std_msgs::Float32>("simu_send_wind_direction",0);
    std_msgs::Float32 msgWind;
    ros::Publisher pub_awind = nh.advertise<std_msgs::Float32>("simu_send_wind_speed",0);
    std_msgs::Float32 msgaWind;
    ros::Publisher pub_Euler = nh.advertise<geometry_msgs::Vector3>("simu_send_euler_angles",0);
    geometry_msgs::Vector3 msgEuler;


    t0 = ros::Time::now().toSec();
    x[0] = 0;
    x[1] = 0;
    x[2] = -M_PI/4;
    x[3] = 0;
    x[4] = 0;
    wind = 0.1;
    awind = 2;
    cmdRudder = 0;
    cmdSail = 0;
    ros::Rate loop_rate(25);
    while (ros::ok()){
        ros::spinOnce();
        f();
        euler();
        set_gps(pubGps,msgGps,x);
        set_imu(pub_imu, msgImu,x);
        set_Euler(pub_Euler,msgEuler,x);
        set_wind(pub_wind,msgWind);
        set_awind(pub_awind,msgaWind);

        loop_rate.sleep();
    }
    //sleep(1);
    ROS_INFO("FIN");
    return 0;
}
