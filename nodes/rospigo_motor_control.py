#!/usr/bin/env python

# ROS imports
import roslib, rospy
from std_msgs.msg import Int32MultiArray

# GoPiGo imports
import gopigo

import atexit

# Topic names
topic_motor1 = "/gopigo/motor1"
topic_motor2 = "/gopigo/motor2"

class Empty:
    def __init__(self):
        pass

def stop_motors():
    gopigo.motor1(0,0)
    gopigo.motor2(0,0)

# Callbacks
def motor1_callback(data):
    direction = data.data[0]
    speed = data.data[1]
    v = gopigo.motor1(direction,speed)
    
def motor2_callback(data):
    direction = data.data[0]
    speed = data.data[1]
    v = gopigo.motor2(direction,speed)

# Motor Subscribers
motor1_sub = rospy.Subscriber(topic_motor1, Int32MultiArray, motor1_callback)
motor2_sub = rospy.Subscriber(topic_motor2, Int32MultiArray, motor2_callback)

# Init and Run
rospy.init_node('rospigo_motor_control', anonymous=True)
atexit.register(stop_motors)
rospy.spin()

'''
To test from command line run:

rostopic pub /gopigo/motor1 std_msgs/Int32MultiArray '{data:[<direction>, <speed>]}'
'''

