#!/usr/bin/env python3

import rospy
from trilobot.msg import Battery_state 
from std_msgs.msg import Empty, String, Int32, Bool
from sensor_msgs.msg import BatteryState
from topics import *

CELL_MIN_VOLTAGE = 3.0
class BatteryBridge():
    def __init__(self):
        rospy.init_node('battery_bridge')
        self.ardu_sub = rospy.Subscriber(topic_battery_raw, Battery_state, self.callback)

        self.bridge_pub = rospy.Publisher(topic_battery, BatteryState, queue_size=1)
        self.need_charge_pub = rospy.Publisher(topic_need_charge, Bool, queue_size=10)

    def callback(self, msg):
        # publishing generic battery info
        battery_msg = BatteryState()
        voltages = [msg.cell1, msg.cell2, msg.cell3, msg.cell4]
        battery_msg.header.stamp = rospy.Time.now()
        battery_msg.voltage = sum(voltages)
        battery_msg.cell_voltage = voltages
        battery_msg.present = True

        #logmsg = ""
        #for cell in voltages:
        #    logmsg += str(cell)
        #    logmsg += " "
        #logmsg += " "
        #logmsg += str(msg.charging)
        #rospy.loginfo(logmsg)

        # publishing the need of charging
        self.bridge_pub.publish(battery_msg)
        need_charge = False
        if min(voltages) < CELL_MIN_VOLTAGE:
            need_charge = True
        self.need_charge_pub.publish(Bool(need_charge))

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    bridge = BatteryBridge()
    bridge.spin()
