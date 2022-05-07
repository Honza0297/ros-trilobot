#!/usr/bin/env python3
# Translating messages from Twist to a 


import rospy
from trilobot.msg import Vel
from geometry_msgs.msg import Twist
import math
from topics import *

cmd_vel = "cmd_vel"
cmd_vel_out = "trilobot/cmd_vel"

pub = None

def callback(msg):
	rospy.loginfo("callback from move base")
	pubmsg = Vel()
	pubmsg.x = min(msg.linear.x, 0.2)
	pubmsg.y = min(msg.linear.y,0.2)
	pubmsg.theta = min(msg.angular.z, math.pi/10)
	pub.publish(pubmsg)

if __name__ == "__main__":
	rospy.init_node("base_controller")
	pub = rospy.Publisher(cmd_vel_out, Vel, queue_size=10)
	priority_sub = rospy.Subscriber()
	sub = rospy.Subscriber(cmd_vel, Twist, callback)
	rospy.spin()
