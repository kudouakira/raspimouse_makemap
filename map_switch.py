#!/usr/bin/env python
#-*- coding: utf-8 -*- 

import sys, time
import itertools
import rospy
import math
import numpy as np
from raspimouse_ros.srv import *
from raspimouse_ros.msg import * 
from std_msgs.msg import UInt16
from std_msgs.msg import Header
from nav_msgs.msg import MapMetaData, OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion
class createmaping(object):
    def __init__(self):
        rospy.init_node('makemap')
        self.__init__rviz()
        self.raw_control(0,0)
        rospy.sleep(1)
        sub = rospy.Subscriber('/raspimouse/lightsensors', LightSensorValues, self.lightsensor_callback, queue_size = 10)
        sub2 = rospy.Subscriber('/raspimouse/switches', Switches, self.switch_callback, queue_size = 10)
        self.sensor = [0,0,0,0]
        self.switch = [0,0,0]
        rospy.sleep(1)
        rospy.loginfo("start")

    def lightsensor_callback(self, msg): #change to map
        if msg.right_forward > 700: self.sensor[2] = True
        else : self.sensor[2] = False
        if msg.right_side > 500: self.sensor[3] = True
        else : self.sensor[3] = False
        if msg.left_forward > 700: self.sensor[1] = True
        else : self.sensor[1] = False
        if msg.left_side > 500: self.sensor[0] = True
        else : self.sensor[0] = False
        #print self.sensor

        self.left_side = msg.left_side
        self.right_side = msg.right_side

    def switch_callback(self, msg): 
        self.switch[0] = msg.front
        self.switch[1] = msg.center
        self.switch[2] = msg.rear
        #print self.switch

    def raw_control(self,left_hz,right_hz):
        #What this here I want to move __init__
        self.pub2 = rospy.Publisher('/raspimouse/motor_raw', LeftRightFreq, queue_size=10)
        L = LeftRightFreq()
        L.left = left_hz
        L.right = right_hz
        self.pub2.publish(L)
        
    def oneframe(self,p,dis, r = 2.4,val = 0,t = 0):
        t=(400*dis)/(2*3.14*r*p) #one frame 18cm
        Val = round(t,1)*10
#        if self.left_side > self.right_side:
        for val in range(int(Val)):
                E=0.2*(self.left_side - 900)
                self.raw_control(p+E,p-E)
                time.sleep(0.1)
                val += 1
 #       elif self.left_side < self.right_side:
 #           for val in range(int(Val)):
 #               E=0.3*(self.right_side - 900)
 #               self.raw_control(p-E,p+E)
 #               time.sleep(0.1)
 #               val += 1

    def turn(self,p, deg, rorl, r=2.4, a=4.8, rl=0): #rorl = -1(right) or 1(left)
        t=(deg*400*a)/(360*r*p)
        if(rl > rorl):
            self.raw_control(p,-p)
            time.sleep(round(t,1))
        elif(rl < rorl):
            self.raw_control(-p,p)
            time.sleep(round(t,1))
        else: print "#u#kin R or L please!!"

    def stop_motor(self):
        self.raw_control(0,0)
        switch_motors(Fales)

    def recognition(self, a=True, b=True, type = 0):
        if a and self.sensor[1] and self.sensor[2] and not b: type = 1
        elif not a and self.sensor[1] and self.sensor[2] and b: type = 2
        elif a and not self.sensor[1] and not self.sensor[2] and b: type = 3
        elif a and self.sensor[1] and self.sensor[2] and b: type = 4
        elif a and not self.sensor[1] and not self.sensor[2] and not b: type = 5
        elif not a and not self.sensor[1] and not self.sensor[2] and b: type = 6
        elif not a and self.sensor[1] and self.sensor[2] and not b: type = 7
        else : print "nothing"
        #print type
        return type

    def run(self, deg=0,x=0,y=0): #main
        self.rviz(self.recognition(),y,x,90+deg) #y,x,deg
        while not rospy.is_shutdown():
            self.pub5.publish(self.bar)
            if self.switch[0]: #straiht
                if   deg == -90:  x+=18
                elif deg == 0: y+=18
                elif deg == 90:x-=18
                elif deg == 180:y-=18
                if x < 0:x = 0
                if x > 72: x = 72
                if y < 0:y = 0
                if y > 72:y = 72
                #error
                a,b =  self.sensor[0],self.sensor[3] # naname
                print a,b
                self.oneframe(400,18)
                self.raw_control(0,0)
                time.sleep(0.5)
                self.rviz(self.recognition(a,b),y,x,90+deg) #y,x,deg
                ##error
                #if deg+90 < 0: deg = 270
                #if deg+90 == 0:x+=3
                #elif deg+90 == 90:y+=3
                #elif deg+90 == 180:x-=3
                #elif deg+90 == 270:y-=3
                #if x < 0:x = 0
                #if x > 15: x = 15
                #if y < 0:y = 0
                #if y > 15:y = 15
                #error
                #self.rviz(self.recognition(),y,x,90+deg) #y,x,deg

            elif self.switch[1]:
                self.turn(300, 90, -1)
                self.raw_control(0,0)
                time.sleep(0.5)
                deg -= 90


            elif self.switch[2]:
                self.turn(300, 90, 1)
                self.raw_control(0,0)
                time.sleep(0.5)
                deg += 90


    def __init__rviz(self):#rviz__init__
        self.pub5 = rospy.Publisher('masaya', OccupancyGrid, queue_size=10)
        self.header = Header()
        self.pose = Pose()
        self.point = Point()
        self.quaternion = Quaternion()
        self.info = MapMetaData()
        self.bar = OccupancyGrid()
        self.header.seq =0
        self.header.frame_id = 'map'
        self.point.x = 0.0
        self.point.y = 0.0 
        self.point.z = 0.0
        self.quaternion.x = 0.0
        self.quaternion.y = 0.0
        self.quaternion.z = 0.0
        self.quaternion.w = 1.0
        self.pose.position = self.point
        self.pose.orientation = self.quaternion
        self.info.width = 72
        self.info.height = 72
        self.info.resolution = 0.1
        self.info.origin = self.pose
        self.map_data()

    def map_data(self): #If you want change map scale, you shoud be cheange this map_data and __init__rviz of height and width and resolution
        self.array = np.array([[0 for i in range(72)]for j in range(72)]) # 15*15 map
        self.map_data = np.array([np.empty((18, 18)) for i in range(9)])
        self.map_data[1] = np.array([[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]])

        self.map_data[2] = np.array([[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]])

        self.map_data[3] = np.array([[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]])

        self.map_data[4] = np.array([[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]])


        self.map_data[5] = np.array([[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]])

        self.map_data[6] = np.array([[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]])

        self.map_data[7] = np.array([[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                                    ,[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]])


    def inversion_matrix(self, type = 3, deg = 270,count = 0):
        a = list(itertools.chain(*self.map_data[type]))
        xy= np.array([[0 for i in range(18)]for j in range(18)])
        if deg == 0:
            for x in range(18):
                for y in range(17,-1,-1):
                    xy[y][x] = a[count]
                    count += 1
        elif deg == 90:
            for y in range(18):
                for x in range(18):
                    xy[y][x] = a[count]
                    count += 1
        elif deg == 180:
            for x in range(17,-1,-1):
                for y in range(18):
                    xy[y][x] = a[count]
                    count += 1
        elif deg == 270:
            for y in range(17,-1,-1):
                for x in range(17,-1,-1):
                    xy[y][x] = a[count]
                    count += 1
        return xy

    def rviz(self, type = 0, i = 0,j = 0, deg = 90):
        self.header.seq += 1
        self.header.stamp = rospy.Time.now()
        self.info.map_load_time = rospy.Time.now()
        self.bar.info = self.info
        self.array[i:i+18,j:j+18] = 50 * self.inversion_matrix(type, deg) #deg
        self.bar.data = list(itertools.chain(*self.array))

if __name__ == '__main__':
    create = createmaping()
    create.raw_control(0,0)
    time.sleep(0.5)
    create.run()
