#!/usr/bin/env python3

import rospy
from trilobot.msg import *
from std_msgs.msg import Empty, String, Int32
from trilobot.msg import Sonar_data
from topics import *
class Starter():
   def __init__(self) 
        
if __name__ == "__main__":
    rospy.init_node('sonars_bridge')
    pub = rospy.Publisher(topic_rosserial_start, Empty, queue_size=10)
    sub = rospy.Subscriber(topic_trilobot_sonar_data, Sonar_data, callback)
    msg = Empty()
    pub.publish(msg)
    rospy.spin() 

    

