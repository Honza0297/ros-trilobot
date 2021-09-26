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
        int diameter = rospy.get_param('~diameter','100')
        self.circle_pub = rospy.Publisher(topic_motors_circle, Motors_circle, queue_size=10)
        self.done_subscrier = # TODO
        rospy.init_node("trilobot_controller", anonymous=True)

    def circle(self, speed):
        rospy.loginfo("Going forward")
        msg = Motors_continual_move()
        msg.speed = speed
        self.continual_pub.publish(msg)

    def check_safe_distance(self, direction, safe_distance=20):
        if direction == "front":
            return self.sonar_data.front > 20
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
    controller.go_forward(25)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        controller.get_sonar_data()
        if not controller.check_safe_distance("front"):
            controller.stop()
            controller.turn_angle(random.randint(-90,90), 25)
            controller.go_forward(25)
