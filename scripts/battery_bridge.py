#!/usr/bin/env python3
"""
Author: Jan Beran
Description: Bridge converts specialspace-effective messages from Arduino
             to standard ROS BatteryState messages. 
"""
import rospy
from trilobot.msg import Battery_state 
from std_msgs.msg import Bool
from sensor_msgs.msg import BatteryState
from topics import *

CELL_MIN_VOLTAGE = 3.2

class BatteryBridge():
    def __init__(self):
        rospy.init_node('battery_bridge', anonymous=True)

        self.ardu_sub = rospy.Subscriber(topic_battery_raw, Battery_state, self.callback)

        self.bridge_pub = rospy.Publisher(topic_battery, BatteryState, queue_size=1)
        self.need_charge_pub = rospy.Publisher(topic_need_charge, Bool, queue_size=10)

    def callback(self, msg):
        # Prepare standard ROS message and fill in interesting values
        # NOTE: Some more values may be filled in as well
        battery_msg = BatteryState()
        voltages = [msg.cell1, msg.cell2, msg.cell3, msg.cell4]
        battery_msg.header.stamp = rospy.Time.now()
        battery_msg.voltage = sum(voltages)
        battery_msg.cell_voltage = voltages
        battery_msg.present = True

        # Publishing the need of charging
        need_charge = False
        if min(voltages) < CELL_MIN_VOLTAGE:
            need_charge = True
        
        # Publish both messages
        self.bridge_pub.publish(battery_msg)
        self.need_charge_pub.publish(Bool(need_charge))

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    bridge = BatteryBridge()
    bridge.spin()
