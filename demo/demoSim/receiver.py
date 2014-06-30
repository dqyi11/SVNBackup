#!/usr/bin/env python

import roslib
import rospy

from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
  
if __name__ == '__main__':

    rospy.init_node('gazebo_listener')
    models = rospy.wait_for_message("/gazebo/model_states", ModelStates)

    for i,name in enumerate(models.name):
	if name == "mobile_base":
            pos = models.pose[i].position
            orientation = models.pose[i].orientation


    #print pos
    print orientation
