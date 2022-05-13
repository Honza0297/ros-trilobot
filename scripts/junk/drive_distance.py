#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from trilobot.msg import Odometry


topic_odometry = "trilobot/odometry"
topic_vel = "cmd_vel"

dst = 0.15 # m
trs, trg, tra = 0,0,0
tls, tlg, tla = 0,0,0
start_set = False

def callback(msg):
    global start_set, trs, trg, tra, tls, tlg, tla
    if not start_set:
        start_set = True
        trs = msg.right
        tls = msg.left
    tra = msg.right
    tla = msg.left
    rospy.loginfo(str(trs) + " " + str(trg) + " " + str(tla))

if __name__ == "__main__":
    rospy.init_node('trilobot_drive_distance', anonymous=True)
    sub = rospy.Subscriber(topic_odometry, Odometry, callback)
    pub = rospy.Publisher(topic_vel, Twist, queue_size=10)
    rate = rospy.Rate(10)
    tlg = dst/0.279*768
    trg = dst/0.279*768
    while not rospy.is_shutdown():
        if tra-trs >= trg or tla-tls >= tlg:
            rospy.loginfo(str(tra-trs) + " " + str(tla-tls))
            exit(0) 
        pubmsg = Twist()
        pubmsg.linear.x = 0.12
        pubmsg.angular.z = 0
        pubmsg.linear.y = 0
        pub.publish(pubmsg)
        rate.sleep()

