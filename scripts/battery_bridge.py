#!/usr/bin/env python3

import rospy
from trilobot.msg import Battery_state 
from std_msgs.msg import Empty, String, Int32
from sensor_msgs.msg import BatteryState
from topics import *

class BatteryBridge():
    def __init__(self):
        rospy.init_node('battery_bridge')
    
        self.ardu_pub = rospy.Publisher(topic_battery_request, Empty, queue_size=1)
        self.ardu_sub = rospy.Subscriber(topic_battery_response, Battery_state, self.callback)

        self.bridge_pub = rospy.Publisher(topic_battery, BatteryState, queue_size=1)

    def callback(self, msg):
        battery_msg = BatteryState()
        voltages = [msg.cell1, msg.cell2, msg.cell3, msg.cell4]
        battery_msg.header.stamp = rospy.Time.now()
        #battery_msg.power_supply_status = 
        battery_msg.voltage = sum(voltages)
        battery_msg.cell_voltage = voltages
        battery_msg.present = True
        logmsg = ""
        for cell in voltages:
            logmsg += str(cell)
            logmsg += " "
        logmsg += " "
        logmsg += str(msg.charging)
        rospy.loginfo(logmsg)

        with open("battery.log", "a") as bf:
            bf.write(logmsg + "\n")
        self.bridge_pub.publish(battery_msg)

    def spin(self):
        rate = rospy.Rate(0.05)
        while not rospy.is_shutdown():
            #rospy.loginfo("Getting battery info")
            #self.ardu_pub.publish(Empty())
            rate.sleep()

if __name__ == "__main__":
    bridge = BatteryBridge()
    bridge.spin()
