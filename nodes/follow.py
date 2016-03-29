#!/usr/bin/env python
from __future__ import division
from optparse import OptionParser
import roslib
import rospy
import numpy as np
import time

from std_msgs.msg import Float32MultiArray, Int32MultiArray

def circdiff(alpha,beta):
    D = np.arctan2(np.sin(alpha-beta),np.cos(alpha-beta))
    return D
    
def circmean(alpha,axis=None):
    mean_angle = np.arctan2(np.mean(np.sin(alpha),axis),np.mean(np.cos(alpha),axis))
    return mean_angle

class Follower(object):
    def __init__(self, nodenum):
        
        self.subPrefObj = rospy.Subscriber('/multi_tracker/' + nodenum + '/prefobj', Float32MultiArray, self.tracked_object_callback)
        
        self.pubMotor1 = rospy.Publisher('/gopigo/motor1', Int32MultiArray, queue_size=3)
        self.pubMotor2 = rospy.Publisher('/gopigo/motor2', Int32MultiArray, queue_size=3)
        
        self.current_angle_estimate = 0
        self.desired_angle = []
        # 7 seconds to turn 180 deg at speed 100
        
        self.speed_gain = 20
        self.angular_gain = 30
        self.angular_velocity = 0
        self.speed = [0]
        self.last_time = time.time()
        
        rospy.init_node('rospigo_follower', anonymous=True)
        rospy.sleep(5)
        
        msg = Int32MultiArray()
        msg.data = [1, 0]
        self.pubMotor1.publish(msg)
        self.pubMotor2.publish(msg)
        

        rospy.spin()
        
    def tracked_object_callback(self, objdata):
        objid, position_x, position_y, velocity_x, velocity_y = objdata.data
        dt = time.time() - self.last_time
        self.last_time = time.time()
        
        degrees_per_second = self.angular_velocity / 100. * 7/np.pi
        self.current_angle_estimate += degrees_per_second*dt
        self.current_angle_estimate = circdiff(self.current_angle_estimate, 0)
        
        speed = np.linalg.norm([velocity_x, velocity_y])
        motorspeed = speed*self.speed_gain
        self.speed.append(motorspeed)
        if len(self.speed) > 5:
            a = self.speed.pop(0)
        motorspeed = np.mean(self.speed)
        
        angle = np.arctan2(velocity_x, velocity_y)
        self.desired_angle.append(angle)
        if len(self.desired_angle) > 5:
            a = self.desired_angle.pop(0)
        angle = circmean(self.desired_angle)
        angle_error = circdiff(angle, self.current_angle_estimate)
        self.angular_velocity = self.angular_gain * angle_error * (np.exp(motorspeed/100.)-1)
        
        print angle, angle_error
        
        if self.angular_velocity > 0:
            motor1 = np.abs(self.angular_velocity)
            motor2 = -1*np.abs(self.angular_velocity)
        else:
            motor2 = np.abs(self.angular_velocity)
            motor1 = -1*np.abs(self.angular_velocity)
            
        motor1 += motorspeed
        motor2 += motorspeed
        
        direction1 = 0
        direction2 = 0
        if motor1 < 0:
            direction1 = 1
            motor1 = np.abs(motor1)
        if motor2 < 0:
            direction2 = 1
            motor2 = np.abs(motor2)
        
        msg1 = Int32MultiArray()
        msg1.data = [direction1, motor1]
        
        msg2 = Int32MultiArray()
        msg2.data = [direction2, motor2]
        
        if 0:
            if motorspeed < 60:
                msg1.data = [1, 0]
                msg2.data = [1, 0]
                self.angular_velocity = 0
        
        self.pubMotor1.publish(msg1)
        self.pubMotor2.publish(msg2)
        
        
#####################################################################################################
    
if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("--nodenum", type="str", dest="nodenum", default='1',
                        help="node number, for example, if running multiple tracker instances on one computer")
    (options, args) = parser.parse_args()
    
    follower = Follower(options.nodenum)
