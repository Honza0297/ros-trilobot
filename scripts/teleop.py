#!/usr/bin/env python3
"""
Author: Jan Beran
Description: Simple teleop node converting messages from teleop node to something realistic for trilobot (mainly speeds).
"""
import rospy
from geometry_msgs.msg import Twist



topic_key = "key_vel"
topic_vel = "cmd_vel"

x,z = 0,0
MAX_SPEED = 0.12
MAX_ANGULAR = 0.3

if __name__ == "__main__":
    rospy.init_node('trilobot_teleop', anonymous=True)
    pub = rospy.Publisher(topic_vel, Twist, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = rospy.wait_for_message(topic_key, Twist, timeout=None)
        if(msg.linear.x > 0):
            x = x+0.01 if x < MAX_SPEED else x
        elif(msg.linear.x < 0):
             x = x-0.01 if x > -MAX_SPEED else x
        elif(msg.linear.x == 0):
            if x < 0:
                x += 0.1
            elif x > 0:
                x -= 0.1
            else: x=0
            if abs(x) < 0.04:
                x = 0

        if(msg.angular.z > 0):
            z = MAX_ANGULAR
        elif(msg.angular.z < 0):
            z = -MAX_ANGULAR
        elif(msg.angular.z == 0):
            z = 0
        pubmsg = Twist()
        pubmsg.linear.x = x
        pubmsg.angular.z = z
        pub.publish(pubmsg)
        rate.sleep()

