#!/usr/bin/env python2
# This Python file uses the following encoding: utf-8
import serial
import rospy
from gps_common.msg import GPSFix
from std_msgs.msg import String


status = 0

time = 0
longitude = 0
latitude = 0
type = 0
nbSat = 0
hdop = 0
altitude = 0

valid = 0
speed,route = 0,0
date =  0
modePositionnement = 0

def trameGpsIsValide(data):
    if (data[0] != '$' ):
        return 0
    res = 0
    for i in data:
        if (i == '*'):
            break
        if (i != ',' and i != '$'):
            res = res ^ ord(i)
    data = data.split(',')
    test = int(data[-1].split('*')[1],16)
    if (test == res):
        return 1
    else:
        return 0

def parseGPGGA(data):
    time = float(data[1][0:2])*3600 + float(data[1][2:4])*60 + float(data[1][4:])
    if data[2] == '':
        return 0,time,0,0,0,0,0,0
    latitude = float(data[2][0:(len(data[2])-7)])+float(data[2][(len(data[2])-7):])/60
    if data[3] == 'S':
        latitude = -latitude
    longitude = float(data[4][0:(len(data[4])-7)])+float(data[4][(len(data[4])-7):])/60
    if data[5] == 'E':
        longitude = -longitude
    type = float(data[6])
    nbSat = float(data[7])
    hdop = float(data[8])
    altitude = float(data[9].split(',')[0])
    return 1,time,latitude,longitude,type,nbSat,hdop,altitude


def parseGPRMC(data):
    time = float(data[1][0:2])*3600 + float(data[1][2:4])*60 + float(data[1][4:])
    if data[3] == '':
        return 0,time,0,0,0,0,0,0,0
    latitude = float(data[3][0:(len(data[3])-7)])+float(data[3][(len(data[3])-7):])/60
    if data[4] == 'S':
        latitude = -latitude
    longitude = float(data[5][0:(len(data[5])-7)])+float(data[5][(len(data[5])-7):])/60
    if data[6] == 'E':
        longitude = -longitude
    valid = data[2]
    speed = float(data[7])*0.514444
    route = float(data[8])
    date = data[9]
    mode = data[-2]
    return 1,time,latitude,longitude,valid,speed,route,date,mode


def publishTrame(pub_trame,trame):
    message = String()
    message.data = trame
    pub_trame.publish(message)


def publishGps(pub_gps,frame):
    message = GPSFix()

    #header
    now = rospy.get_rostime()
    message.header.stamp.secs = now.secs
    message.header.stamp.nsecs = now.nsecs
    message.header.frame_id = '/filter_send_gps'

    #status
    message.status.header.seq = message.header.seq
    message.status.header.stamp.secs = now.secs
    message.status.header.stamp.nsecs = now.nsecs
    message.status.header.frame_id = frame
    message.status.satellites_used = nbSat
    message.status.status = status

    #autres
    message.latitude = latitude
    message.longitude = longitude
    message.altitude = altitude
    message.track = route
    message.speed = speed
    message.time = time
    message.hdop = hdop

    pub_gps.publish(message)




if __name__ == "__main__":

    ser = serial.Serial('/dev/ttyUSB0', 4800)
    rospy.init_node("gps")

    pub_trame = rospy.Publisher('filter_send_trame_gps',String,queue_size = 10)
    pub_gps = rospy.Publisher('filter_send_gps',GPSFix,queue_size = 10)

    while not rospy.is_shutdown():
        trame = ser.readline()
        #print(trame)
        if trameGpsIsValide(trame):
           data = trame.split(',')
           print(data)
           if (data[0] == '$GPGGA'):
               status,time,latitude,longitude,type,nbSat,hdop,altitude = parseGPGGA(data)
               publishGps(pub_gps,'GPGGA')
               print(latitude)
           elif (data[0] == '$GPRMC'):
               status,time,latitude,longitude,valid,speed,route,date,mode = parseGPRMC(data)
               publishGps(pub_gps,'GPRMC')
           publishTrame(pub_trame,trame)









