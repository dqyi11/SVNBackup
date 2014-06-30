#!/usr/bin/env python

import roslib
import rospy
import math
import dubins
import numpy as np

from TurtlebotController import *
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion

class TurtlebotTarget(object):
    
    def __init__(self):
        self.pos = {'x':0.0,'y':0.0,'z':0.0}
        self.orientation = {'roll':0.0,'pitch':0.0,'yaw':0.0}
        self.reached = False

class TurtlebotPlanner(object):
    
    def __init__(self, sampleTime):
        self.sampleTime = sampleTime
        self.currentPos = {'x':0.0,'y':0.0,'z':0.0}
        self.currentOrientation = {'roll':0.0,'pitch':0.0,'yaw':0.0}
        
        self.tIdx = 1
        self.threshold = 0.5
        
        self.wayPoints = []
        self.wayOrientations = []
        self.wayRadius = []
        
        self.currentTarget = None
        rospy.init_node('gazebo_listener')
        
        self.ctrl = TurtlebotController(sampleTime, self)
        
    def update(self):
        models = rospy.wait_for_message("/gazebo/model_states", ModelStates)
        
        for i,name in enumerate(models.name):
            pose_i = models.pose[i]

            if name == "mobile_base":
                self.currentPos['x'] = pose_i.position.x
                self.currentPos['y'] = pose_i.position.y
                self.currentPos['z'] = pose_i.position.z
                
                quatvec = (pose_i.orientation.x, pose_i.orientation.y, pose_i.orientation.z, pose_i.orientation.w)
                euler = euler_from_quaternion(quatvec)

                self.currentOrientation['roll'] = euler[0]
                self.currentOrientation['pitch'] = euler[1]
                self.currentOrientation['yaw'] = euler[2]
                
        if self.reached(self.tIdx) == True:
            self.tIdx = self.tIdx + 1
                
    def control(self):
        self.ctrl.control(self.tIdx)
        
        
    def reached(self, t):  
        if t >= len(self.wayPoints):
            return False
        
        dist = math.sqrt((self.currentPos['x'] - self.wayPoints[t][0])**2+(self.currentPos['y'] - self.wayPoints[t][1])**2) 
        if dist <= self.threshold:
            return True
        return False   
    
    def plan(self, turnRadius):
        q0 = (self.currentPos['x'], self.currentPos['y'], self.currentOrientation['yaw'])
        q1 = (self.currentTarget.pos['x'], self.currentTarget.pos['y'], self.currentTarget.orientation['yaw'])
        self.wayPoints, T = dubins.path_sample(q0, q1, turnRadius, self.sampleTime)        
    
        self.wayOrientations = []
        self.wayRadius = []
        self.wayDotRadius = []
        self.wayDotOrientation = []
        for t in range(len(self.wayPoints)-1):
            deltaT = T[t+1] - T[t]
            deltaX = self.wayPoints[t+1][0] - self.wayPoints[t][0]
            deltaY = self.wayPoints[t+1][1] - self.wayPoints[t][1]
            orientation = np.arctan2(deltaY, deltaX)
            self.wayOrientations.append(orientation)
        self.wayOrientations.append(self.currentTarget.orientation['yaw'])
        for p in self.wayPoints:
            radius = p[0]**2 + p[1]**2
            self.wayRadius.append(radius)
        '''    
        self.wayDotRadius.append(0.0)
        self.wayDotOrientation.append(0.0)
        for t in range(len(self.wayPoints)-1):
            dotRadius = (self.wayRadius[t+1]-self.wayRadius[t])/(T[t+1]-T[t])
            dotOrientation = (self.wayOrientations[t+1]-self.wayOrientations[t])/(T[t+1]-T[t])
            self.wayDotRadius.append(dotRadius)
            self.wayDotOrientation.append(dotOrientation)
        '''    
        self.tIdx = 1
            
        
        
        

