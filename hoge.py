#!/usr/bin/env python
# -*- coding: utf-8 -*- 
import sys, time
import itertools
import rospy
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
        self.raw_control(0,0)
        rospy.sleep(0.5)
        sub = rospy.Subscriber('/raspimouse/lightsensors', LightSensorValues, self.lightsensor_callback)
        self.sensor = [0,0,0,0]

    def lightsensor_callback(self, msg, val = 800): #vel change to map
        if msg.right_forward > val: self.sensor[2] = True
        else : self.sensor[2] = False
        if msg.right_side > val: self.sensor[3] = True
        else : self.sensor[3] = False
        if msg.left_forward > val: self.sensor[1] = True
        else : self.sensor[1] = False
        if msg.left_side > val: self.sensor[0] = True
        else : self.sensor[0] = False
        #print self.sensor

    def raw_control(self,left_hz,right_hz):
        #What this here 
        self.pub2 = rospy.Publisher('/raspimouse/motor_raw', LeftRightFreq, queue_size=10)
        L = LeftRightFreq()
        L.left = left_hz
        L.right = right_hz
        self.pub2.publish(L)
        
    def oneframe(self,p,dis, count = []):
        r=2.4
        t=(400*dis)/(2*3.14*r*p) #one frame 18cm
        self.raw_control(p,p)
        time.sleep(round(t,1))


    def turn(self,p, deg, rorl, r=2.4, a=5.0, rl=0): #rorl = -1(right) or 1(left)
        t=(deg*400*a)/(360*r*p)
        if(rl > rorl):
            self.raw_control(p,-p)
            time.sleep(round(t,1))
        elif(rl < rorl):
            self.aw_control(-p,p)
            timself.sleep(round(t,1))
        else: print "#u#kin R or L please!!"

    def stop_motor(self):
        self.raw_control(0,0)
        switch_motors(Fales)

    def recognition(self, type = 0):
        if self.sensor[0] and self.sensor[1] and self.sensor[2] and not self.sensor[3]: type = 1
        elif not self.sensor[0] and self.sensor[1] and self.sensor[2] and self.sensor[3]: type = 2
        elif self.sensor[0] and not self.sensor[1] and not self.sensor[2] and self.sensor[3]: type = 3
        elif self.sensor[0] and self.sensor[1] and self.sensor[2] and self.sensor[3]: type = 4
        elif self.sensor[0] and not self.sensor[1] and not self.sensor[2] and not self.sensor[3]: type = 5
        elif not self.sensor[0] and not self.sensor[1] and not self.sensor[2] and self.sensor[3]: type = 6
        else : pass #print "nothing"
        print type
        return type



    def run(self):
        self.__init__rviz()
        while not rospy.is_shutdown():
            self.rviz(self.recognition())
        #self.oneframe(400,18)
        #create.raw_control(0,0)
        
        #Jwhile not rospy.is_shutdown():
            #Jself.recognition()
            #pass

    def __init__rviz(self):
        self.pub5 = rospy.Publisher('raspi_mouce', OccupancyGrid, queue_size=10)
        self.header = Header()
        self.pose = Pose()
        self.point = Point()
        self.quaternion = Quaternion()
        self.info = MapMetaData()
        self.bar = OccupancyGrid()
        self.header.seq =0
        self.array = np.array([[0 for i in range(15)]for j in range(15)]) # 15*15 map
        self.map_0 = [[0,0,0],[0,0,0],[0,0,0]]
        self.map_1 = [[100,0,0],[100,0,0],[100,100,100]]
        self.map_2 = [[0,0,100],[0,0,100],[100,100,100]]
        self.map_3 = [[100,0,100],[100,0,100],[100,0,100]]
        self.map_4 = [[100,0,100],[100,0,100],[100,100,100]]
        self.map_5 = [[100,0,0],[100,0,0],[100,0,0]]
        self.map_6 = [[0,0,100],[0,0,100],[0,0,100]]

    def rviz(self, type = 0, i = 0):
        self.header.seq += 1
        self.header.stamp = rospy.Time.now()
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

        self.info.map_load_time = rospy.Time.now()
        self.info.resolution = 5
        self.info.width = 15
        self.info.height = 15
        self.info.origin = self.pose

        self.bar.info = self.info
        if type ==0: self.array[0:3,0:3] = self.map_0
        if type ==1: self.array[0:3,0:3] = self.map_1
        if type ==2: self.array[0:3,0:3] = self.map_2
        if type ==3: self.array[0:3,0:3] = self.map_3
        if type ==4: self.array[0:3,0:3] = self.map_4
        if type ==5: self.array[0:3,0:3] = self.map_5
        if type ==6: self.array[0:3,0:3] = self.map_6
        self.bar.data = list(itertools.chain(*self.array))
        self.pub5.publish(self.bar)

if __name__ == '__main__':
    create = createmaping()
    create.run()
    #create.turn(300, 90, -1)
    #Jturn(300 90, -1) 
