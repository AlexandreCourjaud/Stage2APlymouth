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
ModeBuoy = 0
direction = 0

def sub_gps(msg):
    global x,xgps
    xgps[0] = msg.latitude
    xgps[1] = msg.longitude
    xgps[2] = msg.track
    print(xgps)
    x[0] = 111.11*1000*(msg.latitude-xRef[0])
    x[1] = -111.11*1000*(msg.longitude-xRef[1])*np.cos(xRef[0]*np.pi/180)
    x[2] = msg.track


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
    listPoint = [[],[],[]]
    THIS_FOLDER = os.path.dirname(os.path.abspath(__file__))
    my_file = os.path.join(THIS_FOLDER, 'checkpoint/'+filename)
    file  = open(my_file,'r')
    lines = file.readlines()

    l = lines[1].rstrip('\n').split(',')
    xRef[0] = float(l[0])
    xRef[1] = float(l[1])

    print (lines)
    for i in range(3,len(lines)):
        l = (lines[i].rstrip('\n')).split(',')
        listPoint[0].append(float(l[0]))
        listPoint[1].append(float(l[1]))
        if len(l) > 2:
            listPoint[2].append(float(l[2]))
        else:
            listPoint[2].append(0)

    print(listPoint,xRef)
    return np.array(listPoint),xRef

'''
def control(listPoint,index):
    newPoint = 0
    if index == 0:
        A = listPoint[:2,index].reshape((2,1))
        B = listPoint[:2,index+1].reshape((2,1))
        print(A,B)
        index = index +1
        newPoint = 1
    elif index < len(listPoint[0]):
        A = listPoint[:2,index-1].reshape((2,1))
        B = listPoint[:2,index].reshape((2,1))
        Bcart = np.array([ [111.11*1000*(B[0,0]-xRef[0])],[-111.11*1000*(B[1,0]-xRef[1])*np.cos(xRef[0]*np.pi/180)] ])
        m = x[0:2].reshape((2,1))
        if np.linalg.norm(Bcart-m) < 5:
            index = index +1
            newPoint = 1
    else:
        A = xgps[0:2].reshape((2,1))
        B = listPoint[:2,-1]
        newPoint =1
    return A,B,index,newPoint

'''

def setPoint(listPoint,index):
    if index < len(listPoint[0]):
        A = listPoint[:2,index-1].reshape((2,1))
        B = listPoint[:2,index].reshape((2,1))
    else:
        A = xgps[0:2].reshape((2,1))
        B = listPoint[:2,-1]
    return A,B

def setPointBuoy(listPoint,index):
    global direction
    if ModeBuoy == 0:
        A = xgps[0:2].reshape((2,1))
        B = listPoint[:2,index]

    elif ModeBuoy == 1:
        r = 15
        A = np.array([  [  listPoint[0,index]+r*np.sin(wind)/(111.11*1000)  ] , [  listPoint[1,index]+r*np.cos(wind)/(111.11*1000*np.cos(xRef[0]*np.pi/180) )    ]  ])
        B = np.array([  [  listPoint[0,index]-r*np.sin(wind)/(111.11*1000)  ] , [  listPoint[1,index]-r*np.cos(wind)/(111.11*1000*np.cos(xRef[0]*np.pi/180) )    ]  ])
        if direction == 1:
            A,B = B,A
        m = x[0:2].reshape((2,1))
        Bcart = np.array([ [111.11*1000*(B[0,0]-xRef[0])],[-111.11*1000*(B[1,0]-xRef[1])*np.cos(xRef[0]*np.pi/180)] ])
        if np.linalg.norm(Bcart-m) < 5:
            direction = (direction+1)%2
    return A,B


def control(listPoint,index,timeBuoy):
    A,B = setPoint(listPoint,index)
    if index < len(listPoint[0]) and timeBuoy == 0:
        Bcart = np.array([ [111.11*1000*(B[0,0]-xRef[0])],[-111.11*1000*(B[1,0]-xRef[1])*np.cos(xRef[0]*np.pi/180)] ])
        m = x[0:2].reshape((2,1))
        if np.linalg.norm(Bcart-m) < 15:
            if listPoint[2,index] == 0:
                index = index +1
                #print(index,len(listPoint[0]))
                timeBuoy = 0
            else:
                timeBuoy = time.time()
    if (timeBuoy > 0):
        A,B = setPointBuoy(listPoint,index)
        if (time.time() - timeBuoy > listPoint[2,index]):
            timeBuoy = 0
            index = index+1


    return A,B,index,timeBuoy




if __name__ == "__main__":
    rospy.init_node("checkPointFollow")

    pub_cubeA = rospy.Publisher('control_send_A',Point,queue_size = 10)
    pub_cubeB = rospy.Publisher('control_send_B',Point,queue_size = 10)
    pub_ref = rospy.Publisher('control_send_ref',Point,queue_size = 10)

    mode = rospy.get_param('mode',0)
    ModeBuoy = rospy.get_param('modeBuoy',0)
    file = rospy.get_param('file',"error.txt")


    print file
    print(mode)
    if (mode ==1):
        rospy.Subscriber("simu_send_gps",GPSFix,sub_gps)
        rospy.Subscriber("simu_send_euler_angles",Vector3,sub_euler)
        rospy.Subscriber("simu_send_wind_direction",Float32,sub_wind)
        rospy.Subscriber("simu_send_imu",Imu,sub_imu)

    else:
        rospy.Subscriber("filter_send_gps",GPSFix,sub_gps)
        rospy.Subscriber("filter_send_euler_angles",Vector3,sub_euler)
        rospy.Subscriber("filter_send_wind_direction",Float32,sub_wind)
        rospy.Subscriber("ardu_send_imu",Imu,sub_imu)


    x = np.array([0.0,0.0,0.0])
    xgps = np.array([0.0,0.0,0.0])
    listPoint,xRef = lectureCheckpoint(file)
    index = 1
    buoy = 0
    cubeA = Point()
    cubeB = Point()
    refmsgs = Point()
    refmsgs.x = xRef[0]
    refmsgs.y = xRef[1]
    rate = rospy.Rate(25)
    while not rospy.is_shutdown():

        #A,B,index,newPoint = control(listPoint,index)
        A,B,index,buoy = control(listPoint,index,buoy)
        cubeA.x = A[0]
        cubeA.y = A[1]
        cubeB.x = B[0]
        cubeB.y = B[1]
        pub_cubeA.publish(cubeA)
        pub_cubeB.publish(cubeB)
        pub_ref.publish(refmsgs)
        rate.sleep()
