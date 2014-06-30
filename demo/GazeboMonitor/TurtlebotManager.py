#!/usr/bin/env python

import roslib
import rospy
import math
import numpy as np

from TurtlebotController import *
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion

class TurtlebotManager(object):
    
    def __init__(self, sampleTime):
        
        self.threshold = 2.0
        self.plannedPath = None
        self.ctrl = TurtlebotController(sampleTime)
        self.tIdx = 0
        
        self.currentPos = [0.0, 0.0]
        self.currentOrientation = 0.0        
        
        rospy.init_node('gazebo_listener')
        
        self.scale = 1.0
        self.worldsize = [0, 0]
        
    def reached(self, t):
        
        if self.plannedPath == None:
            return False        
        
        if t >= self.plannedPath.length:
            return False
        
        dist = math.sqrt((self.currentPos[0]-self.plannedPath.waypoints[t][0])**2+(self.currentPos[1]-self.plannedPath.waypoints[t][1])**2)
        if dist <= self.threshold:
            return True
        return False    
    
    
    def loadPath(self, path):
        self.plannedPath = path
        self.tIdx = 0
        self.ctrl.loadPath(path)
        
    
    def control(self):
        self.ctrl.control(self.tIdx)
        
        
    def update(self):
        
        models = rospy.wait_for_message("/gazebo/model_states", ModelStates)
        
        for i,name in enumerate(models.name):
            pose_i = models.pose[i]

            if name == "mobile_base":
                posx = pose_i.position.x
                posy = pose_i.position.y
                self.currentPos[0] = int(posx * self.scale) + self.worldsize[0]/2
                self.currentPos[1] = int(posy * self.scale) + self.worldsize[1]/2                
                
                
                quatvec = (pose_i.orientation.x, pose_i.orientation.y, pose_i.orientation.z, pose_i.orientation.w)
                euler = euler_from_quaternion(quatvec)
                
                self.currentOrientation = euler[2]
                
            self.ctrl.currentPos = self.currentPos
            self.ctrl.currentOrientation = self.currentOrientation
            
        if self.reached(self.tIdx) == True:
            self.tIdx = self.tIdx + 1