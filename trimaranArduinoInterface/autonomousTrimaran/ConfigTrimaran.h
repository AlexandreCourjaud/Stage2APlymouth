
#include <Adafruit_PWMServoDriver.h>
#include <std_msgs/Float32.h>

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
float rudderAngle =77.5;
float maxRudderAngle = 155*3/4;
float minRudderAngle = 155*1/4;

float sailAngle = 0;
float maxSailAngle = 140;
float minSailAngle = 10;

/************** PIN******************************/

#define RUDDER_PIN 12
#define SAIL_PIN   1

// Rc module 

// sail ch2 : 980 -> 1980
// rudder ch1 : 890 -> 1800

#define MAX_CHRUDDER 1900
#define MIN_CHRUDDER 1030

#define MAX_CHSAIL 1970
#define MIN_CHSAIL 1350


/* pin */ 
const int chPinRudder = 4; // channel 1 sur le pin 4
const int chPinSail = 5; // channel 3 sur le pin 5

float chRudder;
float chSail;

unsigned long duration = 10000;



#endif
