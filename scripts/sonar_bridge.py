#!/usr/bin/env python3

import rospy
from trilobot.msg import *
from std_msgs.msg import Empty, String, Int32
from trilobot.srv import SonarsRate, SonarsRateResponse
from trilobot.msg import Sonar_data
import random
import threading

topic_sonars_request = "trilobot/sonars_req"
topic_sonars_response = "trilobot/sonars_resp"
topic_bridge_operation = "trilobot/sonars_bridge"
topic_set_rate = "trilobot/set_sonars_rate"

sonars_rate = 1 
enabled = False

def set_rate_callback(msg):
    global sonars_rate
    # theoretically, max range can be up to 15(incl) for
    # srf measuring time of 65 ms or 14 for 70 ms ( as set of at 25 Sept 2021)
    # 10 is just a safe value
    rospy.loginfo("received new rate: {} of type {}. old rate was {}".format(msg.data, type(msg.data), sonars_rate))
    if msg.data in range(0,11):
        #rospy.loginfo("setting...")
        sonars_rate = msg.data

def handle_sonars_rate(req):
    return SonarsRateResponse(sonars_rate)

def operation_callback(msg):
    global enabled
    if msg.data == "run":
        enabled = True
    elif msg.data == "stop":
        enabled = False
    rospy.loginfo("enabled is {}".format(enabled))

def requester():
    global sonars_rate
    global enabled
    rate = rospy.Rate(sonars_rate)
    req_pub = rospy.Publisher(topic_sonars_request, Empty, queue_size=1)

    while not rospy.is_shutdown():
        rate_val = 1/(rate.sleep_dur.secs*10**9+rate.sleep_dur.nsecs)*(10**9)
        #rospy.loginfo("Rate: {}, sonars_rate: {}".format(rate_val, sonars_rate))
        if rate_val != sonars_rate:
            rate = rospy.Rate(sonars_rate)
        if enabled:
            req_pub.publish(Empty())
        rate.sleep()

def subscriber():
    global sonars_rate
    rate_service = rospy.Service('sonars_rate', SonarsRate, handle_sonars_rate)

    #  enable/disable data request to arduino
    operation_sub = rospy.Subscriber(topic_bridge_operation, String, operation_callback)
    # setting request rate
    rate_sub = rospy.Subscriber(topic_set_rate, Int32, set_rate_callback)

    #rate = rospy.Rate(12) # Hz, max 10 Hz from srf, so this should be OK
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node('sonars_bridge')
    
    requester_thread = threading.Thread(target=requester)
    subscriber_thread = threading.Thread(target=subscriber)

    subscriber_thread.start()
    requester_thread.start()
    

