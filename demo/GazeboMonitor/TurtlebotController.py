#!/usr/bin/env python

import roslib
import rospy

from geometry_msgs.msg import Twist
import numpy as np

class TurtlebotController(object):
    
    def __init__(self, sampleTime):
        self.sampleTime = sampleTime
        self.pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist)

        self.plannedPath = None
        
        self.currentPos = [0.0, 0.0]
        self.currentOrientation = 0.0
        
        self.K1 = 0.5
        self.K2 = 1.0
    
    def loadPath(self, path):
        self.plannedPath = path
        
        
    def control(self, t):
        
        if self.plannedPath == None:
            return
        
        linear_vel = 0.0
        angular_vel = 0.0
        
        if t < self.plannedPath.length:
            
            deltaX = self.plannedPath.waypoints[t][0] - self.currentPos[0]
            deltaY = self.plannedPath.waypoints[t][1] - self.currentPos[1]
            
            expectAngle = np.arctan2(deltaY, deltaX)
            currentAngle = self.currentOrientation
            angular_err = expectAngle - currentAngle
            
            linear_vel = 0.3
            
            expectDeg = expectAngle*180/np.pi
            currentDeg = currentAngle*180/np.pi
            
            errDeg = expectDeg - currentDeg
            
            if errDeg > 180:
                errDeg = errDeg - 360
            elif errDeg < -180:
                errDeg = errDeg + 360
            
            print "expect: " + str(expectDeg)
            print "current: " + str(currentDeg)
            print "err: " + str(errDeg)
            
            if errDeg > 15.0:
                angular_vel = 0.4
            elif errDeg < - 15.0:
                angular_vel = - 0.4
            else:
                angular_vel = 0.0
            #angular_vel = - self.K2*angular_err
        
        print "ctrl " + str(linear_vel) + " : " + str(angular_vel)    
        twist = Twist()
        twist.linear.x = linear_vel
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_vel
        self.pub.publish(twist)
        
    
    