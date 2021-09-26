#!/usr/bin/env python3

import rospy
from trilobot.msg import *
from std_msgs.msg import Empty
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

class Demo_Controller:
    def __init__(self):
        rospy.loginfo("Demo controller initialisation")
        self.continual_pub = rospy.Publisher(topic_motors_continual_move, Motors_continual_move, queue_size=10)
        self.turn_pub = rospy.Publisher(topic_motors_turn, Motors_turn, queue_size=10)
        self.stop_pub = rospy.Publisher(topic_motors_stop, Empty, queue_size=10)
        self.sonar_publisher = rospy.Publisher(topic_sonars_request, Empty, queue_size=10)
        self.sonar_data = Sonar_data()
        rospy.init_node("trilobot_controller", anonymous=True)

    def sonar_callback(self, data):
        self.sonar_data.front = data.front
        self.sonar_data.front_left = data.front_left
        self.sonar_data.front_right = data.front_right
        self.sonar_data.back_left = data.back_left
        self.sonar_data.back_right = data.back_right
        self.sonar_data.back = data.back

    def go_forward(self, speed):
        rospy.loginfo("Going forward")
        msg = Motors_continual_move()
        msg.speed = speed
        self.continual_pub.publish(msg)

    def check_safe_distance(self, direction, safe_distance=20):
        if direction == "front":
            return self.sonar_data.front > safe_distance
        # TODO ostatni smery

    def get_sonar_data(self):
        rospy.loginfo("Getting sonar data: start")
        self.sonar_publisher.publish(Empty())
        self.sonar_data = rospy.wait_for_message(topic_sonars_distance, Sonar_data, timeout=None)
        rospy.loginfo("Getting sonar data: done")


    def stop(self):
        self.stop_pub.publish(Empty())

    def turn_angle(self, angle, speed):
        msg = Motors_turn()
        msg.angle = angle
        msg.speed = speed
        self.turn_pub.publish(msg)

if __name__ == "__main__":
    controller = Demo_Controller()
    controller.go_forward(20)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        controller.get_sonar_data()
        if not controller.check_safe_distance("front"):
            controller.stop()
            controller.turn_angle(180, 25)
            controller.go_forward(20)
