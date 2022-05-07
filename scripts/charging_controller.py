#!/usr/bin/env python3

import rospy
from trilobot.msg import Battery_state 
from std_msgs.msg import Empty, String, Int32, Bool
from sensor_msgs.msg import BatteryState
from topics import *

STATE_INIT = 0
STATE_FETCH_DOCK = 1
STATE_NEED_CHARGE_CHECK = 2
STATE_PREP = 3


class ChargingController():
    def __init__(self) :
        self.supressor = rospy.Publisher(topic_priority_move, Bool, queue_size=10)
        self.init_charge_pos = None
        self.charge_needed = False
        self.state = STATE_INIT
        self.charge_need_checker = rospy.Subscriber(topic_need_charge, Bool, self.ncc) # ncc = need charge callback

    def ncc(self, msg):
        self.charge_needed = msg.data

    def update(self):
        if self.state == STATE_INIT:
            self.state = STATE_FETCH_DOCK
        elif self.state == STATE_FETCH_DOCK:
            self.fetch_dock()
        elif self.state == STATE_NEED_CHARGE_CHECK:
            self.check_charge_need()
        elif self.state == STATE_PREP:
            self.prepare_for_charge_goal()
        elif self.state == STATE_...:
            pass
    
    def fetch_dock(self):
        dock_tf = False
        next_state = STATE_FETCH_DOCK
        # TODO Check whether there is a dock tf present
        if dock_tf:
            self.init_charge_pos = 0 # TODO save pose in map tf 20-30 cm in front of dock
            next_state = STATE_NEED_CHARGE_CHECK
        self.state = next_state

    def check_charge_need(self):
        if self.charge_needed:
            self.state = STATE_PREP

    def prepare_for_charge_goal(self):
        msg = Bool()
        msg.data = True
        self.supressor.publish(msg)



        