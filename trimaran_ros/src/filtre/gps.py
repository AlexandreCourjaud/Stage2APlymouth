#!/usr/bin/env python2

import rospy
import serial
import numpy as np

if __name__ == "__main__":
    rospy.init_node("gps_filter")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        print(ser.readline())




