
import rospy
from trilobot.msg import Vel
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import math
from topics import *


class MoveBridge:
    def __init__(self, priority_mode=False):
        self.priority_move = False
        self.priority_mode = priority_mode

        self.sub = rospy.Subscriber(topic_priority_move, Bool, self.priority_callback)
        self.pub = rospy.Publisher(cmd_vel_out, Vel, queue_size=10)
        self.sub = rospy.Subscriber(cmd_vel if not self.priority_mode else topic_priority_cmd_vel, Twist,
                                    self.callback_standard if not self.priority_mode else self.callback_priority_move)

    def callback_priority_move(self, msg):
        if not self.priority_move:  # quietly do nothing
            return
        pubmsg = Vel()
        pubmsg.x = min(msg.linear.x, 0.2)
        pubmsg.y = min(msg.linear.y, 0.2)
        pubmsg.theta = min(msg.angular.z, math.pi / 10)
        self.pub.publish(pubmsg)

    def callback_standard(self, msg):
        if self.priority_move:
            rospy.logwarn("Priority move in progress")
        pubmsg = Vel()
        pubmsg.x = min(msg.linear.x, 0.2)
        pubmsg.y = min(msg.linear.y, 0.2)
        pubmsg.theta = min(msg.angular.z, math.pi / 10)
        self.pub.publish(pubmsg)

    def priority_callback(self, msg):
        if msg.data:
            self.priority_move = True
        else:
            self.priority_move = False

    def spin(self):
        rospy.spin()