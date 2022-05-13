#!/usr/bin/env python3

import rospy
from trilobot.msg import *
from std_msgs.msg import Empty, String, Int32
from geometry_msgs.msg import Twist

import random
from trilobot.srv import *

topic_cmd_vel = "trilobot/cmd_vel"

topic_sonars_request = "trilobot/sonars_req"
topic_sonars_response = "trilobot/sonars_resp"
topic_set_rate = "trilobot/set_sonars_rate"

topic_bridge_enable = "trilobot/sonars_bridge"

class SonarData():
    def __init__(self):
        self.front = 0
        self.front_right = 0
        self.front_left = 0
        self.back_right = 0
        self.back_left = 0
        self.back = 0
        
sonar_data = {
        "front":0,
        "front_left":0,
        "front_right":0,
        "back_right":0,
        "back_left":0,
        "back":0
        }


def check_rate():
    rate = 0
    rospy.wait_for_service('sonars_rate')
    try:
        sonars_rate = rospy.ServiceProxy('sonars_rate', SonarsRate)
        rate = sonars_rate(Empty())
    except rospy.ServiceException as exc:
        rate = -1
    finally:
        return rate

def data_callback(msg):
    sonar_data["front"] = msg.front
    sonar_data["front_right"] = msg.front_right
    sonar_data["front_left"] = msg.front_left
    sonar_data["back_right"] = msg.back_right
    sonar_data["back_left"] = msg.back_left
    sonar_data["back"] = msg.back
    #rospy.loginfo("data jsou: {}".format(sonar_data.front))
def opposite_dir(direction):
    ret = "front"
    if direction == "front":
       ret = "back" 
    elif direction == "front_right":
        ret = "back_left"
    elif direction == "front_left":
        ret = "back_right"
    elif direction == "back_right":
        ret = "front_left"
    elif direction == "back_left":
        ret = "front_right"
    elif direction == "back":
        ret = "front"
    else:
        ret = "none"
    return ret


def get_direction():
    # Vraci smer, kam jet - OD nejmensi vzdalenosti
    k, v = "none", 500
    sonar_data["front"] = sonar_data["front"]-7
    for key in sonar_data:
        if sonar_data[key] < v:
            k = opposite_dir(key)
            v = sonar_data[key]
    return k,v

def set_Twist_msg(direction):
    # NOTE v pythonu 3.10 je konecne "switch" - jmenuje se to match-case. az bude mit ubuntu python 3.10, pouzit to tu
    msg = Twist()
    if direction == "front":
        msg.linear.x = 0.12
        msg.angular.z = 0
    elif direction == "front_right":
        msg.linear.x = 0.09
        msg.angular.z = 0.3
    elif direction == "front_left":
        msg.linear.x = 0.09
        msg.angular.z = -0.3
    elif direction == "back_right":
        msg.linear.x = -0.09
        msg.angular.z = 0.5
    elif direction == "back_left":
        msg.linear.x = -0.09
        msg.angular.z = -0.5
    elif direction == "back":
        msg.linear.x = -0.12
        msg.angular.z = 0
    else:
        msg.linear.x = 0
        msg.angular.z = 0
    return msg

if __name__ == "__main__":
    sample_rate = 5 # Hz

    rate_pub = rospy.Publisher(topic_set_rate, Int32, queue_size=10)
    bridge_enable_pub = rospy.Publisher(topic_bridge_enable, String, queue_size=1, latch=True)
    motors_pub = rospy.Publisher(topic_cmd_vel, Twist, queue_size=10)
    
    sonar_sub = rospy.Subscriber(topic_sonars_response, Sonar_data, data_callback)
    rospy.init_node("obstacle_avoider_simple")
    
    rate_pub.publish(Int32(sample_rate))

    bridge_enable_msg = String()
    bridge_enable_msg.data = "run"
    bridge_enable_pub.publish(bridge_enable_msg)
    rospy.loginfo("Published enable comd")

    rate = rospy.Rate(sample_rate)
    while not rospy.is_shutdown():
        direction, dist = get_direction()
        twist = set_Twist_msg(direction)
        rospy.loginfo("X: {}, Z: {}".format(twist.linear.x, twist.angular.z))
        motors_pub.publish(twist)
        rate.sleep()





