#include <std_msgs/Float32.h>


#define PIN_WIND A1
#define MAX_WIND 970
#define MIN_WIND 45
float sensorValue =0;
float angleWind =0;
float ref = 0;

std_msgs::Float32 windMsg;
ros::Publisher pubWind("ardu_send_wind_direction",&windMsg);

std_msgs::Float32 windSpeedMsg;
ros::Publisher pubWindSpeed("ardu_send_wind_speed",&windSpeedMsg);

const byte pinAnemo= 2;
int counterAnemo = 0;
unsigned long t0;
unsigned long t1;
double windSpeed;
int validWind = 0;
