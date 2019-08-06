#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point



distance = 0
capBuoy = 0
newData = 0

xRef = [0,0]
x = [0,0]
yaw,pitch,roll = 0,0,0


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
    global distance, capBuoy
    distance = msg.x
    capBuoy = msg.y+yaw
    newData = 1

def sub_ref(msg):
    global xRef
    xRef[0] = msg.x
    xRef[1] = msg.y

if __name__ == "__main__":

    rospy.init_node("poseBuoy")


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

    while not rospy.is_shutdown():
        if newData == 1:
            newData = 0
            posx = x[0] + np.cos(capBuoy)*distance
            posy = x[1] + np.sin(capBuoy)*distance
            print(posx,posy)






