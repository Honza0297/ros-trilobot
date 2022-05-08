#!/usr/bin/env python3
# Translating messages from Twist to a 


import rospy
from trilobot.msg import Vel
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import math
from topics import *
from move_bridge import MoveBridge


if __name__ == "__main__":
	rospy.init_node("base_controller", anonymous=True)
	move_bridge = MoveBridge()
	move_bridge.spin()
