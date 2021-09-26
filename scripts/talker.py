#!/iusr/bin/env python

import rospy
from trilobot.msg import *
from std_msgs.msg import Empty

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
    rospy.init_node("sonar_getter", anonymous=True)
    rate = rospy.Rate(0.1)
    while not rospy.is_shutdown():
        msg = Empty()
        rospy.loginfo("Requesting data from sonars")
        pb.publish(msg)
        rate.sleep()


def talker():
    pb = rospy.Publisher(topic_motors_move, Motors_move, queue_size=10)
    rospy.init_node("talker", anonymous=True);
    rate = rospy.Rate(0.1)
    while not rospy.is_shutdown():
            msg = Motors_move()
            msg.distance = 30
            msg.speed = 30
            rospy.loginfo("Sending msg in shape: distance: {}, speed: {}".format(msg.distance, msg.speed))
            pb.publish(msg)
            rate.sleep()


if __name__ == "__main__":
    try:
        sonar_getter()
        #talker()
    except rospy.ROSInterruptException:
        pass
