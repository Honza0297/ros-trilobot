#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from apriltag_ros.msg import AprilTagDetectionArray

import tf2_ros as tf
from tf2_geometry_msgs import PoseStamped
import tf2_geometry_msgs


import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


from topics import *

# Offset of the charging position, eg. how far "in front of" the docking station should robot go before final docking
CP_OFFSET = 0.1
DOCK_DICOVERY_TIMEOUT = 10 # [s]


STATE_DISCOVERY = 0
STATE_LATENT = 1
STATE_CHARGE_NEED_DETECTED = 2
STATE_FINAL_NAV = 3

class DockNavigator():

	def __init__(self):
		rospy.init_node("dock_navigator", anonymous=True)
		self.rate = rospy.Rate(10)

		self.state = STATE_DISCOVERY
		self.need_charge = False
		self.charging = False

		self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
		self.client.wait_for_server()

		self.dock_discovery_timeout = rospy.Time.now() + rospy.Duration(DOCK_DICOVERY_TIMEOUT) 

		self.cn_sub = rospy.Subscriber(topic_need_charge, Bool, self.nc_callback)
		self.charging_sub = rospy.Subscriber(topic_charging, Bool, self.charge_callback)

		self.dock_position_sub = rospy.Subscriber("tag_detections",AprilTagDetectionArray,self.pos_callback)
		
		# Charging position without time in "dock" frame
		self.charging_position = PoseStamped()
		
		#NOTE: delete once done
		self.temp_pub = rospy.Publisher("temp_dock_pose", PoseStamped, queue_size=10)

		self.buff = tf.Buffer(rospy.Duration(120.0))
		self.tfl = tf.TransformListener(self.buff)

		self.rate = rospy.Rate(10)

	def charge_callback(self, msg):
		self.charging = msg.data

	def nc_callback(self, msg):
		self.need_charge = msg.data

	def pos_callback(self, msg):
		# Actually, I dont do anything with the message - just update my internal info about charging position.
		
		# temp position of the dock in camera_color_optical_frame frame
		#temp_dock = PoseStamped()
		#temp_dock.header = msg.detections[0].pose.header
		#temp_dock.pose = msg.detections[0].pose.pose.pose
		
		charging_position = PoseStamped()
		charging_position.header.frame_id = "dock"
		charging_position.pose.position.x = 0
		charging_position.pose.position.y = 0
		charging_position.pose.position.z = CP_OFFSET
		charging_position.pose.orientation.x = 0
		charging_position.pose.orientation.y = 1
		charging_position.pose.orientation.z = 0
		charging_position.pose.orientation.w = 1



		#TODO: check whether the time warping is needed
		#                               dst,    src  
		tr = self.buff.lookup_transform("map", "dock", rospy.Time.now() - rospy.Duration(0.5), rospy.Duration(0.5))
		charging_position.header.stamp = rospy.Time.now()
		self.charging_position = tf2_geometry_msgs.do_transform_pose(charging_position, tr) 
		self.temp_pub.publish(self.charging_position)

	def spin(self):
		if self.state == STATE_DISCOVERY:
			if rospy.Time.now() > self.dock_discovery_timeout:
				rospy.logerr("Cannot find docking station, turning off...")
				rospy.signal_shutdown(None)
			else:
				rospy.spin()

		elif self.state == STATE_LATENT:
			rospy.spin() # to update dock position
			if self.need_charge:
				self.state = STATE_CHARGE_NEED_DETECTED
		
		elif self.state == STATE_CHARGE_NEED_DETECTED:
			# TODO: set saved charging position as a goal for move base as a service and monitor state
			goal = MoveBaseGoal()
			self.charging_position.header.stamp = rospy.Time.now()
			goal.target_pose = self.charging_position
			self.client.send_goal(goal)
			res = None
			wait = self.client.wait_for_result()
			if not wait:
				rospy.logerr("Action server not available!")
				rospy.signal_shutdown("Action server not available!")
			else:
				res = self.client.get_result()
				rospy.loginfo("Result from move_base: {}".format(res))
			rospy.spin()
			self.state = STATE_FINAL_NAV

		elif self.state == STATE_FINAL_NAV:
			rospy.logerr("Now, i should perform final navigation to the station, but i could not :(")
			# TODO final navigation 


	

if __name__ == "__main__":
	x = DockNavigator()
	rospy.loginfo("Starting dock navigator")
	rospy.spin()
