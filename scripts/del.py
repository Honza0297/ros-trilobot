#!/usr/bin/env python3

import rospy
from std_msgs.msg import Empty


if __name__ == "__main__":
	rospy.init_node("DELE")
	pub = rospy.Publisher("/ignore", Empty, queue_size=10)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		pub.publish(Empty())
		rate.sleep()
