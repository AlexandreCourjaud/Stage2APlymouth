#!/usr/bin/env python2

# This Python file uses the following encoding: utf-8

import rosbag
import os
import rospy
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from gps_common.msg import GPSFix
import time



if __name__ == "__main__":
    filename = "checkPointFollow_2019-07-26-12-39-53.bag"
    THIS_FOLDER = os.path.dirname(os.path.abspath(__file__))
    my_file = os.path.join(THIS_FOLDER, '../../rosbag/rosbag27_07/'+filename)
    bag = rosbag.Bag(my_file)
    print(bag)
    for topic, msg, t in bag.read_messages(topics=['/mode_send_u_rudder','/mode_send_u_sail']):
        print msg.data
        print topic
        print t
    bag.close()
