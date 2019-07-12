#!/usr/bin/env python2


# This Python file uses the following encoding: utf-8
import numpy as np
import rospy
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
import time
import os

def sub_gps(msg):
    global x
    x[0] = msg.x
    x[1] = msg.y
    x[2] = msg.theta



def sub_euler(msg):
    global yaw, pitch,roll
    yaw = msg.x
    pitch = msg.y
    roll = msg.z

def sub_wind(msg):
    global wind
    wind = msg.data


def sub_imu(msg):
    global ax,ay,az,gx,gy,gz
    ax = msg.linear_acceleration.x
    ay = msg.linear_acceleration.y
    az = msg.linear_acceleration.z

    gx = msg.angular_velocity.x
    gy = msg.angular_velocity.y
    gz = msg.angular_velocity.z


def lectureCheckpoint(filename):
    listPoint = [[],[]]
    THIS_FOLDER = os.path.dirname(os.path.abspath(__file__))
    my_file = os.path.join(THIS_FOLDER, 'checkpoint/'+filename)
    file  = open(my_file,'r')
    lines = file.readlines()
    for i in lines:
        l = (i.rstrip('\n')).split(',')
        listPoint[0].append(float(l[0]))
        listPoint[1].append(float(l[1]))
    return np.array(listPoint)

def control(listPoint,index):
    newPoint = 0
    if index == 0:
        A = listPoint[:,index].reshape((2,1))
        B = listPoint[:,index+1].reshape((2,1))
        print(A,B,"ok")
        index = index +1
        newPoint = 1
    elif index < len(listPoint[0]):
        A = listPoint[:,index-1].reshape((2,1))
        B = listPoint[:,index].reshape((2,1))
        m = x[0:2].reshape((2,1))
        if np.linalg.norm(B-m) < 5:
            index = index +1
            newPoint = 1
            print(index)
    else:
        A = x[0:2].reshape((2,1))
        B = listPoint[:,-1]
        newPoint =1
    return A,B,index,newPoint




if __name__ == "__main__":
    rospy.init_node("checkPointFollow")

    pub_cubeA = rospy.Publisher('control_send_A',Point,queue_size = 10)
    pub_cubeB = rospy.Publisher('control_send_B',Point,queue_size = 10)


    rospy.Subscriber("simu_send_gps",Pose2D,sub_gps)
    rospy.Subscriber("simu_send_euler_angles",Vector3,sub_euler)
    rospy.Subscriber("simu_send_wind",Float32,sub_wind)
    rospy.Subscriber("simu_send_imu",Imu,sub_imu)

    '''
    rospy.Subscriber("filter_send_gps",Pose2D,sub_gps)
    rospy.Subscriber("filter_send_euler_angle",Vector3,sub_euler)
    rospy.Subscriber("filter_send_wind",Float32,sub_wind)
    rospy.Subscriber("filter_send_imu",Imu,sub_imu)
    '''


    x = np.array([0,0,0])
    listPoint = lectureCheckpoint("test.txt")
    index = 0
    cubeA = Point()
    cubeB = Point()
    rate = rospy.Rate(25)
    while not rospy.is_shutdown():

        A,B,index,newPoint = control(listPoint,index)
        cubeA.x = A[0]
        cubeA.y = A[1]
        cubeB.x = B[0]
        cubeB.y = B[1]
        pub_cubeA.publish(cubeA)
        pub_cubeB.publish(cubeB)

        rate.sleep()
