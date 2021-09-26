/usr/bin/env python3

import rospy
from trilobot.msg import *
from std_msgs.msg import Empty, UInt16
import random

topic_motors_move = "trilobot/motors/move"
topic_motors_continual_move = "trilobot/motors/continual_move"
topic_motors_turn = "trilobot/motors/turn"
topic_motors_continual_turn = "trilobot/motors/continual_turn"
topic_motors_stop = "trilobot/motors/stop"
topic_motors_confirmation = "trilobot/motors/confirmation"

topic_sonars_request = "trilobot/sonars/request"
topic_sonars_distance = "trilobot/sonars/distance"

def sonar_getter():
    pb = rospy.Publisher(topic_sonars_request, Empty, queue_size=10)

    rate = rospy.Rate(0.1)
    while not rospy.is_shutdown():
        msg = Empty()
        rospy.loginfo("Requesting data from sonars")
        pb.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    sonar_getter()
