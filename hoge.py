#!/usr/bin/env python
# -*- coding: utf-8 -*- 
import sys, time
import rospy
from raspimouse_ros.srv import *
from raspimouse_ros.msg import *
from std_msgs.msg import UInt16

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
        else : print "nothing"



    def run(self):
        #self.oneframe(400,18)
        #create.raw_control(0,0)
        
        while not rospy.is_shutdown():
            self.recognition()
            pass


if __name__ == '__main__':
    create = createmaping()
    create.run()
    #create.turn(300, 90, -1)
    #Jturn(300 90, -1) 
