#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from trilobot.msg import Sonar_data



topic_motors_move = "trilobot/motors/move"
topic_motors_continual_move = "trilobot/motors/continual_move"
topic_motors_turn = "trilobot/motors/turn"
topic_motors_continual_turn = "trilobot/motors/continual_turn"
topic_motors_stop = "trilobot/motors/stop"
topic_motors_confirmation = "trilobot/motors/confirmation"

topic_sonars_request = "trilobot/sonars/request"
topic_sonars_distance = "trilobot/sonars/distance"

def callback(data):
    rospy.loginfo(data.data)

def sonar_callback(data):
    rospy.loginfo("Got sonar data:")
    rospy.loginfo("Front: {}".format(data.front))
    rospy.loginfo("Front right: {}".format(data.front_right))
    rospy.loginfo("Front left: {}".format(data.front_left))
    rospy.loginfo("Back right: {}".format(data.back_right))
    rospy.loginfo("Back left: {}".format(data.back_left))
    rospy.loginfo("Back: {}".format(data.back))

def sonar_listener():
    rospy.loginfo("Starting sonar listener...")

    rospy.init_node("sonar_listener", anonymous=True)
    rospy.Subscriber(topic_sonars_distance, Sonar_data, sonar_callback)

    rospy.spin()

def listener():
    rospy.init_node("listener", anonymous=True)
    rospy.Subscriber(topic_sonars_distance, Sonar_data, sonar_callback) #tohle jsem rozbil :(

    rospy.spin()

if __name__ == "__main__":
    sonar_listener()
    #listener()
