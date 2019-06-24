#include <std_msgs/Float32.h>


#define PIN_WIND A1
#define MAX_WIND 970
#define MIN_WIND 45
float sensorValue =0;
float angleWind =0;
float ref = 0;

std_msgs::Float32 windMsg;
ros::Publisher pubWind("Wind",&windMsg);
