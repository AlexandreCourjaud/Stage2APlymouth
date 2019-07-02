
#include <Adafruit_PWMServoDriver.h>
#include <std_msgs/Float32.h>

#ifndef CONFIG_H
#define CONFIG_H


/*******************************Suscriber*************************************/
void rudderCallBack(const std_msgs::Float32& cmd);
ros::Subscriber<std_msgs::Float32> rudderSub("control_send_u_rudder", &rudderCallBack );


void sailCallBack(const std_msgs::Float32& cmd);
ros::Subscriber<std_msgs::Float32> sailSub("control_send_u_sail", &sailCallBack );




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
float maxRudderAngle = 115;
float minRudderAngle = 25;

float sailAngle = 0;
float maxSailAngle = 140;
float minSailAngle = 10;

/************** PIN******************************/

#define RUDDER_PIN 12
#define SAIL_PIN   1



#endif
