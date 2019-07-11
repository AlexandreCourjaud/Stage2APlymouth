#!/usr/bin/env python2


import numpy as np
import rospy
import time
import os
import matplotlib.pyplot as plt

x = np.array([0,0])
xRef = np.array([50.3755,-4.1402])

def clear(ax):
    plt.pause(0.001)
    plt.cla()
    ax.set_xlim(ax.xmin,ax.xmax)
    ax.set_ylim(ax.ymin,ax.ymax)

def init_figure(xmin,xmax,ymin,ymax):
    fig = plt.figure(0)
    ax = fig.add_subplot(111, aspect='equal')
    ax.xmin=xmin
    ax.xmax=xmax
    ax.ymin=ymin
    ax.ymax=ymax
    clear(ax)
    return ax

ax = init_figure(-150,10,-10,150)

def newpoint(latitude,longitude):
    xnew = 111.11*1000*(latitude-xRef[0])
    ynew = 111.11*1000*(longitude-xRef[1])*np.cos(xRef[0]*np.pi/180)
    ax.plot(xnew,ynew,'.r')



if __name__ == "__main__":
    rospy.init_node("gps")


    THIS_FOLDER = os.path.dirname(os.path.abspath(__file__))
    my_file = os.path.join(THIS_FOLDER, 'logGps/log.txt')
    file = open(my_file,'r')
    lines = file.readlines()
    listPoint = [[],[]]
    for i in lines:
        l = (i.rstrip('\n')).split(' ')
        listPoint[0].append(float(l[0]))
        listPoint[1].append(float(l[1]))
    listPoint = np.array(listPoint)
    for i in range (0,len(listPoint[0])):
        if (listPoint[0,i] != float('nan')) and (listPoint[1,i] != float('nan')):
            newpoint(listPoint[0,i],listPoint[1,i])

    plt.show()
