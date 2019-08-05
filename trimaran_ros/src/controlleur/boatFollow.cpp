#include <ros/ros.h>
#include <cstdlib>
#include <unistd.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <math.h>
#include <gps_common/GPSFix.h>

using namespace std;


// x,y,cap
double x[3] = {0,0,0};
double xRef[2] = {50.695251,-4.236975};

double xCible[3] = {0,0,0};
double yawCible = 0;

double a[2] = {0,0};
double b[2] = {0,0};


void gpsCB(const gps_common::GPSFix msgGps)
{
    x[0] = msgGps.latitude;
    x[1] = msgGps.longitude;
    x[2] = msgGps.track;
}

void cibleCB(const gps_common::GPSFix msgGps)
{
    xCible[0] = msgGps.latitude;
    xCible[1] = msgGps.longitude;
    xCible[3] = msgGps.track;
}

void eulerCibleCB(const geometry_msgs::Vector3 msgEuler){
  yawCible = msgEuler.x;
}

void Control(){
  a[0]= x[0];
  a[1]= x[1];
  b[0]= xCible[0] - 10*cos(yawCible)/(111.11*1000);
  b[1]= xCible[1] + 10*sin(yawCible)/(111.11*1000*cos(xRef[0]*M_PI/180));

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "CapFollow");
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
  string target;
  nh.param<int>("mode", mode,0);
  nh.param<string>("target",target,"2");

  ROS_INFO("mode : %d", mode);
  string topicCible;
  string topicEulerCible;

  string topicGps;


  if (mode == 0){
    topicGps = "filter_send_gps";
    topicCible = "xbee_send_gps_"+target;
    topicEulerCible = "xbee_send_euler"+target;

  }
  else{
    topicGps = "simu_send_gps";
    topicCible = "/boat2/simu_send_gps";
    topicEulerCible = "/boat2/simu_send_euler_angles";
  }

  ros::Subscriber sub_gps  = nh.subscribe(topicGps,0,gpsCB);
  ros::Subscriber sub_Cible = nh.subscribe(topicCible,0,cibleCB);
  ros::Subscriber sub_euler_Cible = nh.subscribe(topicEulerCible,0,eulerCibleCB);

  ros::Rate loop_rate(25);
  while (ros::ok()){
      ros::spinOnce();
      Control();
      msgA.x = a[0];
      msgA.y = a[1];
      pub_A.publish(msgA);

      msgB.x = b[0];
      msgB.y = b[1];
      pub_B.publish(msgB);

      pub_Ref.publish(msgRef);
      loop_rate.sleep();
  }
   //sleep(1);
   return 0;
}
