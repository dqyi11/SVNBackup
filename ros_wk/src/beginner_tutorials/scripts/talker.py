#!/usr/bin/env python
import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    try:
        pub = rospy.Publisher('chatter', String)
        rospy.init_node('talker')
        count = 0
        while not rospy.is_shutdown():
            msgstr = "hello world " + str(count)
            rospy.loginfo(msgstr)
            pub.publish(String(msgstr))
            count+=1
            rospy.sleep(1.0)
    except rospy.ROSInterruptException:
        pass        
