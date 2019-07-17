#include <ros/ros.h>
#include <std_msgs/Float32.h>


int mode = 0;
float cmdRudder=0;
float cmdSail = 0.5;


int newRudder = 0;
int newSail = 0;

void rudderControlCB(const std_msgs::Float32 msgRudder){
    if (mode ==0){
      cmdRudder = msgRudder.data;
      newRudder = 1;
    }
}

void sailControlCB(const std_msgs::Float32 msgSail){
    if (mode ==0){
      cmdSail = msgSail.data;
      newSail = 1;
    }
}

void rudderXbeeCB(const std_msgs::Float32 msgRudder){
    if (mode == 1){
      cmdRudder = msgRudder.data;
      newRudder = 1;
    }
}

void sailXbeeCB(const std_msgs::Float32 msgSail){
    if (mode ==1){
      cmdSail = msgSail.data;
      newSail = 1;
    }
}

void modeXbeeCB(const std_msgs::Float32 msgMode){
    mode = msgMode.data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mode");
  ros::NodeHandle nh;


  ros::Publisher  pub_Rudder = nh.advertise<std_msgs::Float32>("mode_send_u_rudder",0);
  ros::Publisher  pub_Sail   = nh.advertise<std_msgs::Float32>("mode_send_u_sail",0);

  ros::Subscriber sub_sail_control = nh.subscribe("control_send_u_sail",0,sailControlCB);
  ros::Subscriber sub_rudder_control = nh.subscribe("control_send_u_rudder",0,rudderControlCB);

  ros::Subscriber sub_sail_xbee = nh.subscribe("xbee_send_u_sail",0,sailXbeeCB);
  ros::Subscriber sub_rudder_xbee = nh.subscribe("xbee_send_u_rudder",0,rudderXbeeCB);
  ros::Subscriber sub_mode_xbee = nh.subscribe("xbee_send_mode",0,modeXbeeCB);

  std_msgs::Float32 uRudder;
  std_msgs::Float32 uSail;


  ros::Rate loop_rate(10);
  while (ros::ok()){
    ros::spinOnce();

    if (newSail == 1){
      uSail.data = cmdSail;
      pub_Sail.publish(uSail);
      newSail = 0;
    }
    if (newRudder ==1 ){
      uRudder.data = cmdRudder;
      pub_Rudder.publish(uRudder);
      newRudder = 0;
    }

    loop_rate.sleep();
}
//sleep(1);
return 0;

}
