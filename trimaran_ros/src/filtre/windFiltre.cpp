#include <ros/ros.h>
#include <std_msgs/Float32.h>

float windData;
float Gamma = 10;
float windchap= 0;

float alpha = 0;
float beta = 0.2;

float newData = 0;

void ExtendedkalmanFilter(){
  


}


void windCB(const std_msgs::Float32 msgWind){
    windData = msgWind.data;
    newData = 1;
    ExtendedkalmanFilter();
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "windFiltre");
  ros::NodeHandle nh;


  ros::Subscriber sub_Wind = nh.subscribe("ardu_send_windDirection",0,windCB);

  std_msgs::Float32 windMsg;
  ros::Publisher pub_wind = nh.advertise<std_msgs::Float32>("filter_send_windDirection",0);

  ros::Rate loop_rate(25);
  while (ros::ok){
      ros::spinOnce();

      if (newData == 1){
        windMsg.data =  windchap;
        ROS_INFO("%f",windchap);
        pub_wind.publish(windMsg);
        newData = 0;
      }

      loop_rate.sleep();
  }
    //sleep(1);
    return 0;
}
