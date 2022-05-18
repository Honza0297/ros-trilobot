#!/usr/bin/env python3
"""
Author: Jan Beran
Description: ROS node that currently does nothing ( :) ), but once sonars will be used, 
             it will convert trilobot specific message with sonar readings to ROS standard 
             message for sonars (sensor_msgs/Range). 
"""
import rospy
from trilobot.msg import *
from std_msgs.msg import Empty, String, Int32
from trilobot.msg import Sonar_data
from topics import *

#TODO here do the converting
def callback(msg):
    pass
        
if __name__ == "__main__":
    rospy.init_node('sonars_bridge')
    sub = rospy.Subscriber(topic_sonar_data, Sonar_data, callback)
    rospy.spin() 

    

