#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Empty
from trilobot.msg import *
from geometry_msgs.msg import Twist



topic_key_vel = "key_vel"
topic_motors_continual_move = "trilobot/motors/continual_move"
topic_motors_continual_turn = "trilobot/motors/continual_turn"
topic_motors_stop = "trilobot/motors/stop"


class Teleop_Controller:
    def __init__(self):
        rospy.loginfo("Demo controller initialisation")
        rospy.init_node("teleop_listener", anonymous=True)

        self.move_pub = rospy.Publisher(topic_motors_continual_move, Motors_continual_move, queue_size=10)
        self.turn_pub = rospy.Publisher(topic_motors_continual_turn, Motors_continual_turn, queue_size=10)
        self.stop_pub = rospy.Publisher(topic_motors_stop, Empty, queue_size=10)
        
        self.sub = rospy.Subscriber(topic_key_vel, Twist, self.callback)
        
        self.old_linearx = -10
        self.old_angularz = -10

    def go(self, speed):
        rospy.loginfo("Going straight")
        msg = Motors_continual_move()
        msg.speed = speed
        self.move_pub.publish(msg)

    def turn(self, direction, speed):
        rospy.loginfo("Turning {}".format("left" if direction == "l" else "right"))
        msg = Motors_continual_turn()
        msg.speed = speed
        msg.direction = direction
        self.turn_pub.publish(msg)

    def stop(self):
        self.stop_pub.publish(Empty())


    def callback(self, data):
        # New change in the straight direction
        if data.linear.x != self.old_linearx:
            if data.linear.x == 0:
                self.stop()
            elif data.linear.x > 0:
                self.go(30)
            else: # if data.linear.x < 0:
                self.go(-30)
            self.old_linearx = data.linear.x
            return # If there is a change in straight dir, we do not turn

        # New change in the turning request
        if data.angular.z != self.old_angularz:
            if data.angular.z == 0:
                self.stop()
            elif data.angular.z > 0:
                self.turn(108, 30) # 108 means l in ascii... KINDABUG?
            else: # if data.angular.z < 0:
                self.turn(114, 30) # 114 means r in ascii... KINDABUG? When pass 'r', it fails with some type mismatch...
            self.old_angularz = data.angular.z
            return
    
    def listen(self):
        rospy.spin()


def callback_ang(data):
    global oldx, oldy, oldz
    angular = data.angular
    if oldx != angular.x:
       rospy.loginfo("Change X from {} to {}.".format(oldx, angular.x))
       oldx = angular.x

    if oldy != angular.y:
       rospy.loginfo("Change Y from {} to {}.".format(oldy, angular.y))
       oldy = angular.y
 
    if oldz != angular.z:
       rospy.loginfo("Change Z from {} to {}.".format(oldz, angular.z))
       oldz = angular.z

    

def callback(data):
    global oldx, oldy, oldz
    if oldx != data.linear.x:
        rospy.loginfo("Change from {} to {}.".format(oldx, data.linear.x))
        oldx = data.linear.x

    if (oldy != data.linear.y) or (oldz != data.linear.z): 
        rospy.loginfo("Y or Z changed, too!!!")
        oldy = data.linear.y
        oldz = data.linear.z


def listener():
    rospy.loginfo("Starting listener.")
    rospy.init_node("listener", anonymous=True)
    rospy.Subscriber(topic_key_vel, Twist, callback_ang) 
    rospy.spin()

if __name__ == "__main__":
    #listener()
    controller = Teleop_Controller()
    controller.listen()
