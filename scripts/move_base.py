#!/usr/bin/env python3

"""
Author: Jan Beran
Description: ROS node that wraps MoveBridge class (details in move_bridge.py)
"""

import rospy
from topics import *
from move_bridge import MoveBridge


if __name__ == "__main__":
	rospy.init_node("base_controller", anonymous=True)
	move_bridge = MoveBridge()
	move_bridge.spin()
