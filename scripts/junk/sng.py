#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def act():
	client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	client.wait_for_server()

	msg = MoveBaseGoal()
	msg.target_pose.header.frame_id = "base_link"
	msg.target_pose.header.stamp = rospy.Time.now()
	msg.target_pose.pose.position.x = 0.5
	msg.target_pose.pose.orientation.w = 1.0

	client.send_goal(msg)
	client.wait_for_result()
	return

if __name__ == "__main__":
	try:
		rospy.init_node("test_goal_act")
		res = act()
		print(res)
	except rospy.ROSInterruptException:
		print("Interrupted")
