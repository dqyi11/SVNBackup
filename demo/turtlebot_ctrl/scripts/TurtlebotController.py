#!/usr/bin/env python

import roslib
import rospy

from geometry_msgs.msg import Twist
import numpy as np

class TurtlebotController(object):
    
    def __init__(self, sampleTime, planner):
        self.sampleTime = sampleTime
        self.pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist)
        self.tolerance = 1.0
        self.planner = planner
        
        self.K1 = 0.5
        self.K2 = 0.5
        
    def control(self, t):
        
        linear_vel = 0.0
        angular_vel = 0.0
        
        if t < len(self.planner.wayPoints):
            deltaX = self.planner.wayPoints[t][0] - self.planner.currentPos['x']
            deltaY = self.planner.wayPoints[t][1] - self.planner.currentPos['y']
            
            expectAngle = np.arctan2(deltaY, deltaX)
            currentAngle = self.planner.currentOrientation['yaw']
            angular_err = currentAngle - expectAngle
            
            linear_vel = 0.5
            angular_vel = - self.K2*angular_err 
        
        
        twist = Twist()
        twist.linear.x = linear_vel
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_vel
        self.pub.publish(twist)

    