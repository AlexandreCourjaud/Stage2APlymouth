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


xStart = [0,0]
xRef = [0,0]
ModeBuoy = 0
direction = 0

x = np.array([0.0,0.0,0.0])
xgps = np.array([0.0,0.0,0.0])
xBuoy = np.array([0.0,0.0])
gpsready = 0

def sub_gps(msg):
    global x,xgps,gpsready
    xgps[0] = msg.latitude
    xgps[1] = msg.longitude
    xgps[2] = msg.track
    x[0] = 111.11*1000*(msg.latitude-xRef[0])
    x[1] = -111.11*1000*(msg.longitude-xRef[1])*np.cos(xRef[0]*np.pi/180)
    x[2] = msg.track
    gpsready = 1


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

def sub_Buoy(msg):
    global xBuoy
    xBuoy[0] = msg.x
    xBuoy[1] = msg.y


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


def setPoint(listPoint,index):
    global xBuoy,xStart
    if index == 0:
        A = np.array([ [xStart[0]], [xStart[1]] ])
        B = listPoint[:2,0].reshape((2,1))
    elif index < len(listPoint[0]):
        A = listPoint[:2,index-1].reshape((2,1))
        B = listPoint[:2,index].reshape((2,1))
    else:
        A = xgps[0:2].reshape((2,1))
        B = listPoint[:2,-1].reshape((2,1))


    ta = abs(A[0,0]) + abs(A[1,0])
    if (ta == 0):
        A[0,0] = xBuoy[0]
        A[1,0] = xBuoy[1]
    tb = abs(B[0,0]) + abs(B[1,0])
    if (tb == 0):
        B[0,0] = xBuoy[0]
        B[1,0] = xBuoy[1]
    return A,B

def setPointBuoy(listPoint,index):
    global direction,wind
    if ModeBuoy == 0:
        A = xgps[0:2].reshape((2,1))
        B = listPoint[:2,index]

    elif ModeBuoy == 1:
        r = 2
        A = np.array([  [  listPoint[0,index]+r*np.sin(wind)/(111.11*1000)  ] , [  listPoint[1,index]+r*np.cos(wind)/(111.11*1000*np.cos(xRef[0]*np.pi/180) )    ]  ])
        B = np.array([  [  listPoint[0,index]-r*np.sin(wind)/(111.11*1000)  ] , [  listPoint[1,index]-r*np.cos(wind)/(111.11*1000*np.cos(xRef[0]*np.pi/180) )    ]  ])

        if direction == 1:
            A,B = B,A
        m = x[0:2].reshape((2,1))
        Bcart = np.array([ [111.11*1000*(B[0,0]-xRef[0])],[-111.11*1000*(B[1,0]-xRef[1])*np.cos(xRef[0]*np.pi/180)] ])
        Acart = np.array([ [111.11*1000*(A[0,0]-xRef[0])],[-111.11*1000*(A[1,0]-xRef[1])*np.cos(xRef[0]*np.pi/180)] ])
        BA = Acart-Bcart
        BC = m-Bcart
        scal = BA[0,0]*BC[0,0] + BA[1,0]*BC[1,0]
        if scal < 0 :
            direction = (direction+1)%2

    elif ModeBuoy == 2:
        r = 2
        point = np.array([  [  listPoint[0,index]-r*np.sin(wind)/(111.11*1000), listPoint[0,index]+r*np.sin(wind)/(111.11*1000),listPoint[0,index]-0.6*r*np.sin(wind+np.pi/2)/(111.11*1000)] ,
                            [  listPoint[1,index]-r*np.cos(wind)/(111.11*1000*np.cos(xRef[0]*np.pi/180) ), listPoint[1,index]+r*np.cos(wind)/(111.11*1000*np.cos(xRef[0]*np.pi/180) ),listPoint[1,index]-0.6*r*np.cos(wind+np.pi/2)/(111.11*1000*np.cos(xRef[0]*np.pi/180) )  ]  ])

        point[0] = point[0]+0.3*r*np.sin(wind+np.pi/2)/(111.11*1000)
        point[1] = point[1]+0.3*r*np.cos(wind+np.pi/2)/(111.11*1000*np.cos(xRef[0]*np.pi/180) )
        A = point[:,direction].reshape((2,1))
        B = point[:,(direction+1)%3].reshape((2,1))
        m = x[0:2].reshape((2,1))
        Bcart = np.array([ [111.11*1000*(B[0,0]-xRef[0])],[-111.11*1000*(B[1,0]-xRef[1])*np.cos(xRef[0]*np.pi/180)] ])
        Acart = np.array([ [111.11*1000*(A[0,0]-xRef[0])],[-111.11*1000*(A[1,0]-xRef[1])*np.cos(xRef[0]*np.pi/180)] ])
        BA = Acart-Bcart
        BC = m-Bcart
        scal = BA[0,0]*BC[0,0] + BA[1,0]*BC[1,0]
        if scal < 0 :
            direction = (direction+1)%3

    elif ModeBuoy == 3:
        r = 2
        point = np.array([  [  listPoint[0,index]-r*np.sin(wind)/(111.11*1000),listPoint[0,index]-0.4*r*np.sin(wind+np.pi/2)/(111.11*1000), listPoint[0,index]+r*np.sin(wind)/(111.11*1000),listPoint[0,index]-0.6*r*np.sin(wind+np.pi/2)/(111.11*1000)] ,
                            [  listPoint[1,index]-r*np.cos(wind)/(111.11*1000*np.cos(xRef[0]*np.pi/180) ),listPoint[1,index]-0.4*r*np.cos(wind+np.pi/2)/(111.11*1000*np.cos(xRef[0]*np.pi/180) ) , listPoint[1,index]+r*np.cos(wind)/(111.11*1000*np.cos(xRef[0]*np.pi/180) ),listPoint[1,index]-0.6*r*np.cos(wind+np.pi/2)/(111.11*1000*np.cos(xRef[0]*np.pi/180) )  ]  ])

        point[0] = point[0]+0.3*r*np.sin(wind+np.pi/2)/(111.11*1000)
        point[1] = point[1]+0.3*r*np.cos(wind+np.pi/2)/(111.11*1000*np.cos(xRef[0]*np.pi/180) )
        A = point[:,direction].reshape((2,1))
        B = point[:,(direction+1)%4].reshape((2,1))
        m = x[0:2].reshape((2,1))
        Bcart = np.array([ [111.11*1000*(B[0,0]-xRef[0])],[-111.11*1000*(B[1,0]-xRef[1])*np.cos(xRef[0]*np.pi/180)] ])
        Acart = np.array([ [111.11*1000*(A[0,0]-xRef[0])],[-111.11*1000*(A[1,0]-xRef[1])*np.cos(xRef[0]*np.pi/180)] ])
        BA = Acart-Bcart
        BC = m-Bcart
        scal = BA[0,0]*BC[0,0] + BA[1,0]*BC[1,0]
        if scal < 0 :
            direction = (direction+1)%4

    elif ModeBuoy == 4:
        r = 2.5
        h = 1.5
        convLong = 1/(111.11*1000*np.cos(xRef[0]*np.pi/180) )
        convLat  =1/(111.11*1000)
        p1 = r*np.sin(wind)
        p2 = r*np.cos(wind)
        p3 = h*np.sin(wind +np.pi/2)
        p4 = h*np.cos(wind +np.pi/2)
        point = np.array([ [    listPoint[0,index] - (p1-p3)*convLat,    listPoint[0,index] + (p1-p3)*convLat,     listPoint[0,index] + (p1+p3)*convLat,    listPoint[0,index] - (p1+p3)*convLat ],
                            [   listPoint[1,index] - (p2-p4)*convLong,   listPoint[1,index] + (p2-p4)*convLong,    listPoint[1,index] + (p2+p4)*convLong,   listPoint[1,index] - (p2+p4)*convLong] ])
        A = point[:,direction].reshape((2,1))
        B = point[:,(direction+1)%4].reshape((2,1))
        m = x[0:2].reshape((2,1))
        Bcart = np.array([ [111.11*1000*(B[0,0]-xRef[0])],[-111.11*1000*(B[1,0]-xRef[1])*np.cos(xRef[0]*np.pi/180)] ])
        Acart = np.array([ [111.11*1000*(A[0,0]-xRef[0])],[-111.11*1000*(A[1,0]-xRef[1])*np.cos(xRef[0]*np.pi/180)] ])
        BA = Acart-Bcart
        BC = m-Bcart
        scal = BA[0,0]*BC[0,0] + BA[1,0]*BC[1,0]
        if scal < 0 :
            direction = (direction+1)%4

    return A,B


def control(listPoint,index,timeBuoy):
    A,B = setPoint(listPoint,index)
    if index < len(listPoint[0]) and timeBuoy == 0:
        m = x[0:2].reshape((2,1))
        Bcart = np.array([ [111.11*1000*(B[0,0]-xRef[0])],[-111.11*1000*(B[1,0]-xRef[1])*np.cos(xRef[0]*np.pi/180)] ])
        Acart = np.array([ [111.11*1000*(A[0,0]-xRef[0])],[-111.11*1000*(A[1,0]-xRef[1])*np.cos(xRef[0]*np.pi/180)] ])
        BA = Acart-Bcart
        BC = m-Bcart
        scal = BA[0,0]*BC[0,0] + BA[1,0]*BC[1,0]
        if scal < 0 :
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

    pub_cubeA = rospy.Publisher('control_send_line_begin',Pose2D,queue_size = 10)
    pub_cubeB = rospy.Publisher('control_send_line_end',Pose2D,queue_size = 10)
    pub_ref = rospy.Publisher('control_send_ref',Vector3,queue_size = 10)

    mode = rospy.get_param('mode',0)
    ModeBuoy = rospy.get_param('modeBuoy',0)
    file = rospy.get_param('file',"error.txt")

    rospy.Subscriber("filter_send_buoy",Pose2D,sub_Buoy)
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



    listPoint,xRef = lectureCheckpoint(file)
    index = 0
    buoy = 0
    cubeA = Pose2D()
    cubeB = Pose2D()
    refmsgs = Vector3()
    refmsgs.x = xRef[0]
    refmsgs.y = xRef[1]
    wind = 0


    gpsready = 0
    while gpsready == 0 or ((xgps[0] - xRef[0])**2 + (xgps[1] - xRef[1])**2) > 100:
        time.sleep(0.01)
        pub_ref.publish(refmsgs)

    xStart[0],xStart[1] = xgps[0],xgps[1]
    print(xStart)


    rate = rospy.Rate(25)
    while not rospy.is_shutdown():
        if abs(xgps[0])>0.01:
            A,B,index,buoy = control(listPoint,index,buoy)
            cubeA.x = A[0]
            cubeA.y = A[1]
            cubeB.x = B[0]
            cubeB.y = B[1]
            pub_cubeA.publish(cubeA)
            pub_cubeB.publish(cubeB)
        pub_ref.publish(refmsgs)
        rate.sleep()
