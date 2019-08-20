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


xRef = [0,0]

xKeeping = [0,0]


x = np.array([0.0,0.0,0.0])
xgps = np.array([0.0,0.0,0.0])

def sub_gps(msg):
    global x,xgps
    xgps[0] = msg.latitude
    xgps[1] = msg.longitude
    xgps[2] = msg.track
    x[0] = 111.11*1000*(msg.latitude-xRef[0])
    x[1] = -111.11*1000*(msg.longitude-xRef[1])*np.cos(xRef[0]*np.pi/180)
    x[2] = msg.track


if __name__ == "__main__":
    rospy.init_node("checkPointFollow")


    pub_ref = rospy.Publisher('control_send_ref',Vector3,queue_size = 10)
    rospy.Subscriber("filter_send_gps",GPSFix,sub_gps)


    rate = rospy.Rate(25)
    while not rospy.is_shutdown():

        rate.sleep()
