#!/usr/bin/env python
import roslib
import rospy

from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates


class RobotStateReader(object):

    def __init__(self):
        
        rospy.init_node('RobotStateReader')
        self.posX = 0
        self.posY = 0
        self.posZ = 0
        
    def UpdateState(self):
        
        self.models = rospy.wait_for_message("/gazebo/model_states", ModelStates)
        for i,name in enumerate(self.models.name):
            if name == "mobile_base":
                self.posX = self.models.pose[i].position.x
                self.posY = self.models.pose[i].position.y
                self.posZ = self.models.pose[i].position.z
        
        
        
        
