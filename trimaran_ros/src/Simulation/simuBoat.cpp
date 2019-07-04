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


using namespace std;
using namespace glm;


double wind,awind;
double cmdRudder, cmdSail;

double delta_s;
double p[10] = {0.1,1,6000,1000,2000,1,1,2,300,10000};

//x,y,cap,v,vitesse rotation,
double x[5] = {0,0,0,0,0};
double xdot[5] = {0,0,0,0,0};

double t0;

vec2 cubeA[2] = {0,5};
vec2 cubeB[2] = {30,30};

void windCB(const std_msgs::Float32 msgWind){
    wind = msgWind.data;
}

void rudderCB(const std_msgs::Float32 msgRudder){
    cmdRudder = msgRudder.data;
}

void sailCB(const std_msgs::Float32 msgSail){
    cmdSail = msgSail.data;
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

void set_gps(ros::Publisher pub_gps, geometry_msgs::Pose2D msgGps, double x[5]){
    msgGps.x = x[0];
    msgGps.y = x[1];
    msgGps.theta = x[2];
    pub_gps.publish(msgGps);
}

void set_Euler(ros::Publisher pub_Euler, geometry_msgs::Vector3 msgEuler, double x[5]){
    msgEuler.x = x[2];
    msgEuler.y = 0;
    msgEuler.z = 0;
}

void set_wind(ros::Publisher pub_wind, std_msgs::Float32 msgWind){
    msgWind.data = wind;
}

void set_marker_boat(ros::Publisher vis_pub, visualization_msgs::Marker marker, double x[5]){
       marker.header.frame_id = "bateau1";
       marker.header.stamp = ros::Time();
       marker.ns = "simuBoat";
       marker.id = 0;
       marker.type = visualization_msgs::Marker::MESH_RESOURCE;
       marker.action = visualization_msgs::Marker::ADD;
       marker.pose.position.x = 0;
       marker.pose.position.y = 0;
       marker.pose.position.z = 0;
       tf::Quaternion q;
       q.setRPY(0, 0, 0);
       tf::quaternionTFToMsg(q, marker.pose.orientation);
       marker.scale.x = 1;
       marker.scale.y = 1;
       marker.scale.z = 1;
       marker.color.a = 1.0; // Don't forget to set the alpha!
       marker.color.r = 1.0;
       marker.color.g = 1.0;
       marker.color.b = 1.0;
       //only if using a MESH_RESOURCE marker type:
       marker.mesh_resource = "package://trimaran_ros/meshs/boat.dae";
       vis_pub.publish( marker );
}


void set_marker_rudder(visualization_msgs::Marker marker, ros::Publisher vis_pub)
{
    marker.header.frame_id = "rudder";
    marker.header.stamp = ros::Time();
    marker.ns = "rudder";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    tf::quaternionTFToMsg(q, marker.pose.orientation);
    marker.scale.x = 3;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0;
    marker.color.b = 0;
    //only if using a MESH_RESOURCE marker type:
    //marker.mesh_resource = "package://tp2/meshs/turret.dae";
    vis_pub.publish( marker );
}


void set_marker_sail(visualization_msgs::Marker marker, ros::Publisher vis_pub)
{
    marker.header.frame_id = "sail";
    marker.header.stamp = ros::Time();
    marker.ns = "sail";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    tf::quaternionTFToMsg(q, marker.pose.orientation);
    marker.scale.x = 3.0;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 1.0;
    //only if using a MESH_RESOURCE marker type:
    //marker.mesh_resource = "package://tp2/meshs/turret.dae";
    vis_pub.publish( marker );
}

void set_marker_wind(visualization_msgs::Marker marker, ros::Publisher vis_pub)
{
    marker.header.frame_id = "wind";
    marker.header.stamp = ros::Time();
    marker.ns = "wind";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    tf::quaternionTFToMsg(q, marker.pose.orientation);
    marker.scale.x = 3.0;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0;
    marker.color.g = 1.0;
    marker.color.b = 0;
    //only if using a MESH_RESOURCE marker type:
    //marker.mesh_resource = "package://tp2/meshs/turret.dae";
    vis_pub.publish( marker );
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "simuBoat");
    ros::NodeHandle nh;

    ros::Subscriber sub_sail = nh.subscribe("control_send_u_sail",0,sailCB);
    ros::Subscriber sub_rudder = nh.subscribe("control_send_u_rudder",0,rudderCB);
    ros::Subscriber sub_wind = nh.subscribe("filter_send_wind_direction",0,windCB);


    ros::Publisher  pub_imu = nh.advertise<sensor_msgs::Imu>("simu_send_imu",0);
    sensor_msgs::Imu msgImu;
    ros::Publisher pubGps = nh.advertise<geometry_msgs::Pose2D>("simu_send_gps",0);
    geometry_msgs::Pose2D msgGps;
    ros::Publisher pub_wind = nh.advertise<std_msgs::Float32>("simu_send_wind",0);
    std_msgs::Float32 msgWind;
    ros::Publisher pub_Euler = nh.advertise<geometry_msgs::Vector3>("simu_send_euler_angle",0);
    geometry_msgs::Vector3 msgEuler;


    ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 0 );
    ros::Publisher chatter_pub = nh.advertise<geometry_msgs::PoseStamped>("send_boat", 0);
    geometry_msgs::PoseStamped msg;
    visualization_msgs::Marker marker;
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.child_frame_id = "bateau1";
    transformStamped.header.frame_id = "map";

    ros::Publisher vis_pub_rudder = nh.advertise<visualization_msgs::Marker>("visualization_marker_rudder", 0 );
    tf2_ros::TransformBroadcaster br_rudder;
    geometry_msgs::TransformStamped transformStamped_rudder;
    visualization_msgs::Marker marker_rudder;
    transformStamped_rudder.child_frame_id = "rudder";
    transformStamped_rudder.header.frame_id = "bateau1";

    ros::Publisher vis_pub_sail = nh.advertise<visualization_msgs::Marker>("visualization_marker_sail", 0 );
    tf2_ros::TransformBroadcaster br_sail;
    geometry_msgs::TransformStamped transformStamped_sail;
    visualization_msgs::Marker marker_sail;
    transformStamped_sail.child_frame_id = "sail";
    transformStamped_sail.header.frame_id = "bateau1";

    ros::Publisher vis_pub_wind = nh.advertise<visualization_msgs::Marker>("visualization_marker_wind", 0 );
    tf2_ros::TransformBroadcaster br_wind;
    geometry_msgs::TransformStamped transformStamped_wind;
    visualization_msgs::Marker marker_wind;
    transformStamped_wind.child_frame_id = "wind";
    transformStamped_wind.header.frame_id = "map";

    t0 = ros::Time::now().toSec();
    x[0] = 0;
    x[1] = 0;
    x[2] = 0;
    x[3] = 1;
    x[4] = 0;
    wind = 0;
    awind = 2;
    cmdRudder = -0.1;
    cmdSail = 1;
    ros::Rate loop_rate(25);
    while (ros::ok){
        ros::spinOnce();
        f();
        euler();
        set_gps(pubGps,msgGps,x);
        set_imu(pub_imu, msgImu,x);
        set_Euler(pub_Euler,msgEuler,x);
        set_wind(pub_wind,msgWind);

        //cout << x[0] << " "<< x[1] << " "<< x[2] << endl;
        /***************** Visualisation ***************/

        msg.pose.position.x = x[0];
        msg.pose.position.y = x[1];
        msg.header.stamp = ros::Time::now();
        tf::Quaternion q;
        q.setRPY(0, 0, x[2]);
        tf::quaternionTFToMsg(q, msg.pose.orientation);
        chatter_pub.publish(msg);

        set_marker_boat( vis_pub,marker,x);
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.transform.translation.x = x[0];
        transformStamped.transform.translation.y = x[1];
        tf::quaternionTFToMsg(q, transformStamped.transform.rotation);
        br.sendTransform(transformStamped);

        q.setRPY(0, 0, cmdRudder+M_PI);
        set_marker_rudder(marker_rudder, vis_pub_rudder);
        transformStamped_rudder.header.stamp = ros::Time::now();
        transformStamped_rudder.transform.translation.x = -1;
        transformStamped_rudder.transform.translation.y = 0;
        tf::quaternionTFToMsg(q, transformStamped_rudder.transform.rotation);
        br_rudder.sendTransform(transformStamped_rudder);

        q.setRPY(0, 0, delta_s+M_PI);
        set_marker_sail(marker_sail, vis_pub_sail);
        transformStamped_sail.header.stamp = ros::Time::now();
        transformStamped_sail.transform.translation.x = 3;
        transformStamped_sail.transform.translation.y = 0;
        transformStamped_sail.transform.translation.z = 2;
        tf::quaternionTFToMsg(q, transformStamped_sail.transform.rotation);
        br_sail.sendTransform(transformStamped_sail);


        q.setRPY(0, 0, wind);
        set_marker_wind(marker_wind, vis_pub_wind);
        transformStamped_wind.header.stamp = ros::Time::now();
        transformStamped_wind.transform.translation.x = x[0]+10;
        transformStamped_wind.transform.translation.y = x[1];
        transformStamped_wind.transform.translation.z = 2;
        tf::quaternionTFToMsg(q, transformStamped_wind.transform.rotation);
        br_wind.sendTransform(transformStamped_wind);

        /*************************************/


        loop_rate.sleep();
    }
    //sleep(1);
    return 0;
}
