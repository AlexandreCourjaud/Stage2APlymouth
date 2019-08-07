#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from gps_common.msg import GPSFix
import matplotlib.pyplot as plt
from matplotlib.pyplot import *
import numpy as np
from filter_lib import *
import time


distance = 0
capBuoy = 0
newData = 0

xRef = [0,0]
x = [0,0,0]
xgps = [0,0,0]
yaw,pitch,roll = 0,0,0

def sawtooth(x):
    return (x+np.pi)%(2*np.pi)-np.pi   # or equivalently   2*arctan(tan(x/2))

#################################################
################################################


def sub_gps(msg):
    global x,xgps
    xgps[0] = msg.latitude
    xgps[1] = msg.longitude
    xgps[2] = msg.track
    x[0] = 111.11*1000*(msg.latitude-xRef[0])
    x[1] = -111.11*1000*(msg.longitude-xRef[1])*np.cos(xRef[0]*np.pi/180)
    x[2] = msg.track


def sub_euler(msg):
    global yaw, pitch,roll
    yaw = msg.x
    pitch = msg.y
    roll = msg.z

def sub_buoy(msg):
    global distance, capBuoy, newData, vect_capBuoy, vect_temps
    distance = msg.x+ np.random.randn()*3
    capBuoy = msg.y +np.random.randn()/3 +yaw
    newData = 1
    t = time.time()
    vect_capBuoy = np.array([vect_capBuoy[1],capBuoy,sawtooth(capBuoy-vect_capBuoy[1])])
    vect_temps = np.array([vect_temps[1],t,t-vect_temps[1]])
    vect_distanceBuoy = np.array([vect_distanceBuoy[1],distance,distance - vect_distanceBuoy[1]])


def sub_ref(msg):
    global xRef
    xRef[0] = msg.x
    xRef[1] = msg.y

############################################################
############################################################

vect_temps = np.array([0,0,0])
vect_capBuoy = np.array([0,0,0])
vect_distanceBuoy = np.array([0,0,0])

if __name__ == "__main__":

    rospy.init_node("poseBuoy")

    posX = []
    posY = []
    Lerror = []
    time = []
    buoy = [50.6955,-4.237];
    buoyCart = [0,0]


    mode = rospy.get_param('mode',0)
    print(mode)
    if (mode ==1):
        rospy.Subscriber("simu_send_gps",GPSFix,sub_gps)
        rospy.Subscriber("simu_send_euler_angles",Vector3,sub_euler)
        rospy.Subscriber("simu_send_buoy",Vector3,sub_buoy)

    else:
        rospy.Subscriber("filter_send_gps",GPSFix,sub_gps)
        rospy.Subscriber("filter_send_euler_angles",Vector3,sub_euler)
        rospy.Subscriber("camera_send_buoy",Vector3,sub_buoy)

    rospy.Subscriber("control_send_ref",Point,sub_ref)

    P0 = 10*np.eye(3)
    Q = 0.028**2*np.eye(3)#0.028
    R = 0.01*np.eye(3)
    EKF_yaw   = Extended_kalman_filter(np.zeros((3,1)),P0,f,F,h,H,Q,R)

    t0 = rospy.get_rostime()

    while not rospy.is_shutdown():

        plt.xlim((-1,50))
        plt.ylim((-10,10))



        t1 = rospy.get_rostime()
        if newData == 1:
            newData = 0


            '''
            filtre passe bas
            '''
            '''
            posx = x[0] + np.cos(capBuoy)*distance
            posy = x[1] + np.sin(capBuoy)*distance
            posX.append(posx)
            posY.append(posy)
            if len(posX)>100:
               del(posX[0])
            if len(posY)>100:
               del(posY[0])
            bx = np.mean(posX)
            by = np.mean(posY)
            lat = bx/(111.11*1000)+ xRef[0]
            longi = -by/(111.11*1000*np.cos(xRef[0]*np.pi/180))+xRef[1]

            buoyCart[0] = 111.11*1000*(buoy[0]-xRef[0])
            buoyCart[1] = -111.11*1000*(buoy[1]-xRef[1])*np.cos(xRef[0]*np.pi/180)
            error = np.sqrt((bx-buoyCart[0])**2 + (by-buoyCart[1])**2)
            Lerror.append(error)
            '''
            '''
            filtre kalmann
            '''

            z = np.array([[np.cos(vect_capBuoy[1])],[np.sin(vect_capBuoy[1])],[1]])
            [x,P] = EKF_yaw.EKF_step(vect_capBuoy[2],z)



            time.append(t1.secs-t0.secs)
            plt.plot(time,Lerror)
            print(t1.secs-t0.secs,error)
            plt.pause(0.000000001)
            plt.cla()






