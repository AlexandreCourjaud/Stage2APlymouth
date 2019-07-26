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

double xRef[2] = {50.695326,-4236554};

double wind,awind;
double cmdRudder, cmdSail;

double delta_s;

//x,y,cap,v,w: vitesse rotation
double x[5] = {0,0,0,0,0};
double yaw = 0;
double pitch =0;
double roll =0;


vec2 cubeA = {-10,0};
vec2 cubeB = {10,0};

/**********************************************************************/
void awindCB(const std_msgs::Float32 msgaWind){
  awind = msgaWind.data;
}

void windCB(const std_msgs::Float32 msgWind){
    wind = msgWind.data;
}

void rudderCB(const std_msgs::Float32 msgRudder){
    cmdRudder = msgRudder.data;
}

void sailCB(const std_msgs::Float32 msgSail){
    cmdSail = msgSail.data;

    double theta = yaw;
    double v = x[3];
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
}

void cubeACB(const geometry_msgs::Point msgA){

    cubeA[0] = 111.11*1000*(msgA.x-xRef[0]);
    cubeA[1] = -111.11*1000*(msgA.y-xRef[1])*cos(xRef[0]*M_PI/180);
    ROS_INFO("A : %f, %f",cubeA[0],cubeA[1]);
}

void cubeBCB(const geometry_msgs::Point msgB){

    cubeB[0] = 111.11*1000*(msgB.x-xRef[0]);
    cubeB[1] = -111.11*1000*(msgB.y-xRef[1])*cos(xRef[0]*M_PI/180);
    ROS_INFO("B : %f, %f",cubeB[0],cubeB[1]);
}

void refCB(const geometry_msgs::Point msgRef){
  xRef[0] = msgRef.x;
  xRef[1] = msgRef.y;
}

void eulerCB(const geometry_msgs::Vector3 msgEuler){
  yaw = msgEuler.x;
  pitch = msgEuler.y;
  roll = msgEuler.z;
}

void gpsCB(const gps_common::GPSFix msgGps){
  x[0] = 111.11*1000*(msgGps.latitude-xRef[0]);
  x[1] = -111.11*1000*(msgGps.longitude-xRef[1])*cos(xRef[0]*M_PI/180);
  x[2] = msgGps.track;
  x[3] = msgGps.speed;
}
/***************************************************************************/

void set_marker_boat(ros::Publisher vis_pub, visualization_msgs::Marker marker){
       marker.header.frame_id = "bateau1";
       marker.header.stamp = ros::Time();
       marker.ns = "simuBoat";
       marker.id = 0;
       marker.type = visualization_msgs::Marker::MESH_RESOURCE;
       marker.action = visualization_msgs::Marker::ADD;
       marker.pose.position.x = 0;
       marker.pose.position.y = -1.2;
       marker.pose.position.z = -1;
       tf::Quaternion q;
       q.setRPY(M_PI/2, 0, M_PI/2);
       tf::quaternionTFToMsg(q, marker.pose.orientation);
       marker.scale.x = 0.001;
       marker.scale.y = 0.001;
       marker.scale.z = 0.001;
       marker.color.a = 1.0; // Don't forget to set the alpha!
       marker.color.r = 1.0;
       marker.color.g = 1.0;
       marker.color.b = 1.0;
       //only if using a MESH_RESOURCE marker type:
       marker.mesh_resource = "package://trimaran_ros/meshs/boat.STL";
       vis_pub.publish( marker );
}


void set_marker_rudder(visualization_msgs::Marker marker, ros::Publisher vis_pub)
{
    marker.header.frame_id = "rudder";
    marker.header.stamp = ros::Time();
    marker.ns = "rudder";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 1;
    marker.pose.position.y = 0.1;
    marker.pose.position.z = -2;
    tf::Quaternion q;
    q.setRPY(M_PI/2, 0, -M_PI/2);
    tf::quaternionTFToMsg(q, marker.pose.orientation);
    marker.scale.x = 0.003;
    marker.scale.y = 0.003;
    marker.scale.z = 0.003;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0;
    marker.color.b = 0;
    //only if using a MESH_RESOURCE marker type:
    marker.mesh_resource = "package://trimaran_ros/meshs/rudder.STL";
    vis_pub.publish( marker );
}


void set_marker_sail(visualization_msgs::Marker marker, ros::Publisher vis_pub)
{
    marker.header.frame_id = "sail";
    marker.header.stamp = ros::Time();
    marker.ns = "sail";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 2.7;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    tf::Quaternion q;
    q.setRPY(M_PI/2, 0, -M_PI/2);
    tf::quaternionTFToMsg(q, marker.pose.orientation);
    marker.scale.x = 0.001;
    marker.scale.y = 0.001;
    marker.scale.z = 0.001;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 1.0;
    //only if using a MESH_RESOURCE marker type:
    marker.mesh_resource = "package://trimaran_ros/meshs/sail.STL";
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

void set_marker_A(visualization_msgs::Marker marker, ros::Publisher vis_pub)
{
    marker.header.frame_id = "A";
    marker.header.stamp = ros::Time();
    marker.ns = "A";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    tf::quaternionTFToMsg(q, marker.pose.orientation);
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0;
    //only if using a MESH_RESOURCE marker type:
    //marker.mesh_resource = "package://tp2/meshs/turret.dae";
    vis_pub.publish( marker );
}

void set_marker_B(visualization_msgs::Marker marker, ros::Publisher vis_pub)
{
    marker.header.frame_id = "B";
    marker.header.stamp = ros::Time();
    marker.ns = "B";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    tf::quaternionTFToMsg(q, marker.pose.orientation);
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0;
    marker.color.b = 1.0;
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
    ros::Subscriber sub_ref = nh.subscribe("control_send_ref",0,refCB);
    ros::Subscriber sub_euler = nh.subscribe("ardu_send_euler_angles",0,eulerCB);
    ros::Subscriber sub_gps = nh.subscribe("filter_send_gps",0,gpsCB);
    ros::Subscriber sub_awind = nh.subscribe("ardu_send_wind_speed",0,awindCB);

    ros::Subscriber sub_euler_simu = nh.subscribe("simu_send_euler_angles",0,eulerCB);
    ros::Subscriber sub_gps_sime = nh.subscribe("simu_send_gps",0,gpsCB);
    ros::Subscriber sub_wind_simu = nh.subscribe("simu_send_wind_direction",0,windCB);
    ros::Subscriber wub_awind_simu = nh.subscribe("simu_send_wind_speed",0,awindCB);


    ros::Subscriber sub_A = nh.subscribe("control_send_A",0,cubeACB);
    ros::Subscriber sub_B = nh.subscribe("control_send_B",0,cubeBCB);

    ros::Subscriber sub_gps_xBee = nh.subscribe("xbee_send_gps_1",0,gpsCB);
    ros::Subscriber sub_euler_xbee = nh.subscribe("xbee_send_euler_angles_1",0,eulerCB);


    ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 0 );
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


    ros::Publisher vis_pub_A = nh.advertise<visualization_msgs::Marker>("visualization_marker_A", 0 );
    tf2_ros::TransformBroadcaster br_A;
    geometry_msgs::TransformStamped transformStamped_A;
    visualization_msgs::Marker marker_A;
    transformStamped_A.child_frame_id = "A";
    transformStamped_A.header.frame_id = "map";

    ros::Publisher vis_pub_B = nh.advertise<visualization_msgs::Marker>("visualization_marker_B", 0 );
    tf2_ros::TransformBroadcaster br_B;
    geometry_msgs::TransformStamped transformStamped_B;
    visualization_msgs::Marker marker_B;
    transformStamped_B.child_frame_id = "B";
    transformStamped_B.header.frame_id = "map";

    ros::Rate loop_rate(25);
    while (ros::ok()){
        ros::spinOnce();

        tf::Quaternion q;

        q.setRPY(roll, pitch, yaw);
        set_marker_boat( vis_pub,marker);
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.transform.translation.x = x[0];
        transformStamped.transform.translation.y = x[1];
        tf::quaternionTFToMsg(q, transformStamped.transform.rotation);
        br.sendTransform(transformStamped);

        q.setRPY(0, 0, cmdRudder+M_PI);
        set_marker_rudder(marker_rudder, vis_pub_rudder);
        transformStamped_rudder.header.stamp = ros::Time::now();
        transformStamped_rudder.transform.translation.x = 0;
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



        q.setRPY(0, 0, 0);
        set_marker_A(marker_A, vis_pub_A);
        transformStamped_A.header.stamp = ros::Time::now();
        transformStamped_A.transform.translation.x = cubeA[0];
        transformStamped_A.transform.translation.y = cubeA[1];
        transformStamped_A.transform.translation.z = 0;
        tf::quaternionTFToMsg(q, transformStamped_A.transform.rotation);
        br_A.sendTransform(transformStamped_A);

        q.setRPY(0, 0, 0);
        set_marker_B(marker_B, vis_pub_B);
        transformStamped_B.header.stamp = ros::Time::now();
        transformStamped_B.transform.translation.x = cubeB[0];
        transformStamped_B.transform.translation.y = cubeB[1];
        transformStamped_B.transform.translation.z = 0;
        tf::quaternionTFToMsg(q, transformStamped_B.transform.rotation);
        br_B.sendTransform(transformStamped_B);

        loop_rate.sleep();
    }
    //sleep(1);
    ROS_INFO("FIN");
    return 0;
}

