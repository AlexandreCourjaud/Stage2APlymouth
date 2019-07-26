#include <SoftwareSerial.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>


#define CMPS_GET_ANGLE16 0x13
#define CMPS_GET_PITCH 0x14
#define CMPS_GET_ROLL 0x15
#define CMPS_GET_ACCEL  0x20
#define CMPS_GET_GYRO 0x21



unsigned char high_byte, low_byte;
float pitchCmps, rollCmps, headingCmps;
unsigned int angle16;


unsigned char x_high_byte, x_low_byte;
unsigned char y_high_byte,y_low_byte;
unsigned char z_high_byte,z_low_byte;
float axC,ayC,azC;
float gxC,gyC,gzC;


sensor_msgs::Imu imuCmpsMsg;
ros::Publisher pubImuCmps("ardu_send_imu",&imuCmpsMsg);


geometry_msgs::Vector3 eulerMsg;
ros::Publisher pubEuler("ardu_send_euler_angles", &eulerMsg);

float refImu = 0;
