#!/usr/bin/env python

import sys, time
import rospy
from raspimouse_ros.srv import *
from raspimouse_ros.msg import *
from std_msgs.msg import UInt16

def raw_control(left_hz,right_hz):
    pub = rospy.Publisher('/raspimouse/motor_raw', LeftRightFreq, queue_size=10)

    if not rospy.is_shutdown():
        d = LeftRightFreq()
        d.left = left_hz
        d.right = right_hz
        pub.publish(d)

def lightsensor_callback(data):
    lightsensors.left_side = data.left_side
    lightsensors.right_side = data.right_side
    lightsensors.left_forward = data.left_forward
    lightsensors.right_forward = data.right_forward

def oneframe(p,dis):
    r=2.4
    t=(400*dis)/(2*3.14*r*p)
    raw_control(p,p)
    time.sleep(round(t,1))

def turn(p, deg, rorl): #rorl = -1(right) or 1(left)
    r=2.4
    a=5.0
    rl=0
    t=(deg*400*a)/(360*r*p)
    if(rl > rorl):
        raw_control(p,-p)
        time.sleep(round(t,1))
    elif(rl < rorl):
        raw_control(-p,p)
        time.sleep(round(t,1))
    else:
        print "##cki# R or L please!!"


def stop_motor():
    raw_control(0,0)
    switch_motors(Fales)

if __name__ == '__main__':
    rospy.init_node('one_frame')
#    sub = rospy.Subscriber('/raspimouse/lightsensors', LightSensorValues, lightsensor_callback)
    raw_control(0,0)
    time.sleep(0.5)
    oneframe(400)
    turn(300, 90, -1) 
    raw_control(0,0)
