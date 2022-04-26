#!/usr/bin/env python3

import rospy
from trilobot.msg import *
from std_msgs.msg import Empty, String, Int32
from trilobot.msg import Sonar_data
from topics import *

#TODO zkonvertovat data na sonar zpravy dle ROS dokumentace
def callback(msg):
    pass
        
if __name__ == "__main__":
    rospy.init_node('sonars_bridge')
    sub = rospy.Subscriber(topic_trilobot_sonar_data, Sonar_data, callback)
    rospy.spini() 

    

