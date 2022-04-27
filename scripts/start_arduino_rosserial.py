#!/usr/bin/env python3

import rospy
from trilobot.msg import *
from trilobot.msg import Battery_state
from topics import *

class Starter():
   def __init__(self):
   		self.pub = rospy.Publisher(topic_rosserial_start, Empty, queue_size=10)
        self.control_sub = rospy.Subscriber(topic_battery_raw, Battery_state, self.callback)
        self.arduino_up = False

    def callback(self, msg):
    	self.arduino_up = True

    def spin(self):
    	rate = rospy.Rate(2):
    	while not rospy.is_shutdwn():
    		self.pub.publish(Empty())
    		rate.sleep()

if __name__ == "__main__":
    rospy.init_node('rosserial_arduino_starter')
    starter = Starter()
    starter.spin() 

    

