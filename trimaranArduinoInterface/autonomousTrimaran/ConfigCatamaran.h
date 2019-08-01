#include <Adafruit_PWMServoDriver.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>

#ifndef CONFIG_H
#define CONFIG_H


/*******************************Suscriber*************************************/
void rudderCallBack(const std_msgs::Float32& cmd);
ros::Subscriber<std_msgs::Float32> rudderSub("mode_send_u_rudder", &rudderCallBack );


void sailCallBack(const std_msgs::Float32& cmd);
ros::Subscriber<std_msgs::Float32> sailSub("mode_send_u_sail", &sailCallBack );




/*******************************Actuator**************************************/
#define NB_ACTUATOR 2

#define ACTUATOR_RUDDER    0
#define ACTUATOR_SAIL    1

/****************Configuration*******************/
#define MIN_PULSE_WIDTH       650
#define MAX_PULSE_WIDTH       2350
#define DEFAULT_PULSE_WIDTH   1500
#define FREQUENCY             50

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
float rudderAngle =70;
float maxRudderAngle = 25;
float minRudderAngle = 115;

float sailAngle = 22;
float maxSailAngle = 30;
float minSailAngle = 93;

/************** PIN******************************/

#define RUDDER_PIN 12
#define SAIL_PIN   1


// rc module 




#define MAX_CHRUDDER 1950
#define MIN_CHRUDDER 1080

#define MAX_CHSAIL 1950
#define MIN_CHSAIL 1300

unsigned long timerRc;

/* pin rudder, sail*/ 
const int chPinRudder = 4; // channel 1 sur le pin 4
const int chPinSail = 5; // channel 3 sur le pin 5

float chRudder;
float chSail;

unsigned long duration = 3000;
geometry_msgs::Vector3 rcMsg;
ros::Publisher pubRc("ardu_send_RC",&rcMsg);



#endif
