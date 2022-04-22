#!/usr/bin/env python3
# Translating messages from Twist to a 


import rospy
from trilobot.msg import Vel
from geometry_msgs.msg import Twist

cmd_vel = "cmd_vel"
cmd_vel_out = "trilobot/cmd_vel"

pub = None

def callback(msg):
	rospy.loginfo("callback from move base")
	pubmsg = Vel()
	pubmsg.x = msg.linear.x
	pubmsg.y = msg.linear.y
	pubmsg.theta = msg.angular.z
	pub.publish(pubmsg)

if __name__ == "__main__":
	rospy.init_node("base_controller")
	pub = rospy.Publisher(cmd_vel_out, Vel, queue_size=10)
	sub = rospy.Subscriber(cmd_vel, Twist, callback)
	rospy.spin()
