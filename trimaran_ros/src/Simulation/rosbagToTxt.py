#!/usr/bin/env python2

# This Python file uses the following encoding: utf-8
import numpy as np
import rospy
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from gps_common.msg import GPSFix
import time
import os

def sub_gps(msg):
    global latitude,longitude
    latitude = msg.latitude
    longitude = msg.longitude


def sub_euler(msg):
    global yaw,pitch,roll
    yaw = msg.x
    pitch = msg.y
    roll = msg.z

def sub_wind(msg):
    global wind
    wind = msg.data

def sub_rudder(msg):
    global rudder,newData
    newData = 1
    rudder = msg.data

def sub_sail(msg):
    global sail
    sail = msg.data


def sub_A(msg):
    global ax,ay
    ax = msg.x
    ay = msg.y

def sub_B(msg):
    global bx,by
    bx = msg.x
    by = msg.y

latitude,longitude = 0,0
yaw,pitch,roll = 0,0,0
wind = 0
newData = 0
rudder,sail = 0,0
ax,ay = 0,0
bx,by = 0,0
xRef,yRef = 50.695326,-4.236554



if __name__ == "__main__":
    rospy.init_node("checkPointFollow")


    rospy.Subscriber("filter_send_gps",GPSFix,sub_gps)
    #rospy.Subscriber("ardu_send_euler_angles",Vector3,sub_euler)
    rospy.Subscriber("filter_send_euler_angles",Vector3,sub_euler)
    rospy.Subscriber("ardu_send_wind_direction",Float32,sub_wind)
    rospy.Subscriber("mode_send_u_rudder",Float32,sub_rudder)
    rospy.Subscriber("mode_send_u_sail",Float32,sub_sail)
    rospy.Subscriber("control_send_A",Point,sub_A)
    rospy.Subscriber("control_send_B",Point,sub_B)

    filename = "TestCapFollow_07_26_15-15-28.txt"
    THIS_FOLDER = os.path.dirname(os.path.abspath(__file__))
    my_file = os.path.join(THIS_FOLDER, '../../rosbag/rosbagTxt/'+filename)
    file  = open(my_file,'w')

    rate = rospy.Rate(25)
    while not rospy.is_shutdown():

        if newData == 1:
            newData = 0
            line = str(latitude)+','+str(longitude)+','+str(yaw)+','+str(pitch)+','+str(roll)+','+str(wind)+','+str(rudder)+','+str(sail)+','+str(ax)+','+str(ay)+','+str(bx)+','+str(by)+','+str(xRef)+','+str(yRef)+'\n'
            file.write(line)



        rate.sleep()
    file.close()
