#!/usr/bin/env python

import roslib
#roslib.load_manifest('turtlebot_teleop')
import rospy
import random
import time
import signal

from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
import sys, select, termios, tty


def exitCheck(signum, frame):
    signal.signal(signal.SIGINT, orignal_sigint)
    try:
        if raw_input("\nQuit? (y/n)>").lower().startswith('y'):
            sys.exit(1)
    except KeyboardInterupt:
        sys.exit(1)
        
    signal.signal(signal.SIGINT, exitCheck)


if __name__=="__main__":
    
    original_sigint = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, exitCheck)
        
    rospy.init_node('turtlebot_ctrl')
    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist)

    x = 0
    th = 0
    status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0
    idxcount = 0
    key = 'i'

    
    twist = Twist()
    twist.linear.x = control_speed; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
    pub.publish(twist)

    time.sleep(0.1)

    #print("loop: {0}".format(count))
    #print("target: vx: {0}, wz: {1}".format(target_speed, target_turn))
    #print("publihsed: vx: {0}, wz: {1}".format(twist.linear.x, twist.angular.z))


        
    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    pub.publish(twist)
    time.sleep(0.1)
        
        


