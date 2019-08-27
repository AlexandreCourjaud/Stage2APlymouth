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
newRef = 0
xRef = [29.8678033333,121.53904]
trueBuoy = [29.867288,121.538464]
trueBuoyCart = [111.11*1000*(trueBuoy[0]-xRef[0]), -111.11*1000*(trueBuoy[1]-xRef[1])*np.cos(xRef[0]*np.pi/180)]

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
    global distance, capBuoy, newData
    if msg.x != -999.0:
        distance = msg.x
        capBuoy = msg.y
        newData = 1


def sub_ref(msg):
    global xRef,newRef
    newRef = 1
    xRef[0] = msg.x
    xRef[1] = msg.y

############################################################
############################################################


if __name__ == "__main__":

    rospy.init_node("poseBuoy")

    fpbx = 0
    fpby = 0
    posX = [trueBuoyCart[0]]
    posY = [trueBuoyCart[1]]


    buoy = [0,0];
    buoyCart = [0,0]
    buoyLat = 0
    buoyLong = 0
    '''
    noFiltrelistTime = []
    nbPoint = []
    Error = []
    medianError = []
    meanError = []
    fpbError= []
    zero = []
    '''

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

    rospy.Subscriber("control_send_ref",Vector3,sub_ref)
    pub_buoy = rospy.Publisher('filter_send_buoy',Pose2D,queue_size = 10)
    msgBuoy = Pose2D()
    t0 = rospy.get_rostime()
    time.sleep(3)
    while not rospy.is_shutdown():

        #plt.xlim((-1,100))
        #plt.ylim((-1,15))


        time.sleep(0.1)
        t1 = rospy.get_rostime()
        if newData == 1 and newRef == 1:
            newData = 0
            #listTime.append(t1.secs-t0.secs)
            #nbPoint.append(len(listTime))
            angle = capBuoy + yaw
            posx = x[0] + np.cos(angle)*distance
            posy = x[1] + np.sin(angle)*distance
            buoyCart[0] = 111.11*1000*(buoy[0]-xRef[0])
            buoyCart[1] = -111.11*1000*(buoy[1]-xRef[1])*np.cos(xRef[0]*np.pi/180)
            #print("mesure",posx,posy)
            #print("true",trueBuoyCart[0],trueBuoyCart[1])
            if (abs(buoy[0] - trueBuoyCart[0])<30) and (abs(buoy[1] - trueBuoyCart[1])<30 ):
                posX.append(posx)
                posY.append(posy)


            if len(posX)>10000:
               del(posX[0])
            if len(posY)>10000:
               del(posY[0])




        '''
        no filtre
        '''

        '''
        bx = posx
        by = posy
        error = np.sqrt((bx-buoyCart[0])**2 + (by-buoyCart[1])**2)
        noFiltreError.append(error)
        plt.plot(nbPoint,noFiltreError,'g')

        zero.append(0)
        plt.plot(nbPoint,zero,'black')
        '''

        '''
        filtre median
        '''


        bx = np.median(posX)
        by = np.median(posY)
        #error = np.sqrt((bx-buoyCart[0])**2 + (by-buoyCart[1])**2)
        #medianError.append(error)
        #plt.plot(nbPoint,medianError,'b')

        '''
        Moyenne
        '''

        '''
        bx = np.mean(posX)
        by = np.mean(posY)
        error = np.sqrt((bx-buoyCart[0])**2 + (by-buoyCart[1])**2)
        meanError.append(error)
        plt.plot(nbPoint,meanError,'r')
        '''


        ''' passe bas'''

        '''
        fpbx = 0.90*fpbx + 0.10*posx
        fpby = 0.90*fpby + 0.10*posy
        error = np.sqrt((fpbx-buoyCart[0])**2 + (fpby-buoyCart[1])**2)
        fpbError.append(error)
        plt.plot(nbPoint,fpbError,'y')
        '''

        lat = bx/(111.11*1000)+ xRef[0]
        longi = -by/(111.11*1000*np.cos(xRef[0]*np.pi/180))+xRef[1]


        msgBuoy.x = lat
        msgBuoy.y = longi
        #pub_buoy.publish(msgBuoy)



        #print(error)
        #plt.pause(0.000000001)
        #plt.cla()






