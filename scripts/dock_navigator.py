#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from apriltag_ros.msg import AprilTagDetectionArray
import tf2_ros as tf
from tf2_geometry_msgs import PoseStamped
import tf2_geometry_msgs

from topics import *

# Offset of the charging position, eg. how far "in front of" the docking station should robot go before final docking
CP_OFFSET = 0.3


class DockNavigator():

	def __init__(self):
		rospy.init_node("dock_navigator", anonymous=True)
		

		self.cn_sub = rospy.Subscriber(topic_need_charge, Bool, self.nc_callback)
		self.dock_position_sub = rospy.Subscriber("tag_detections",AprilTagDetectionArray,self.pos_callback)
		
		# Charging position without time in "dock" frame
		self.charging_position = PoseStamped()
		self.charging_position.header.frame_id = "dock"
		self.charging_position.pose.position.x = 0
		self.charging_position.pose.position.y = 0
		self.charging_position.pose.position.z = CP_OFFSET
		self.charging_position.pose.orientation.x = 0
		self.charging_position.pose.orientation.y = 0
		self.charging_position.pose.orientation.z = 1
		self.charging_position.pose.orientation.w = 1

		# Dock position in map frame
		# It is updated until it is impossible
		self.dock_position = PoseStamped()
		self.dock_position.header.frame_id = "map"

		#NOTE: delete once done
		self.temp_pub = rospy.Publisher("temp_dock_pose", PoseStamped, queue_size=10)


		self.buff = tf.Buffer(rospy.Duration(120.0))
		self.tfl = tf.TransformListener(self.buff)

		self.rate = rospy.Rate(10)

	def nc_callback(self, msg):
		self.need_charge = msg.data

	def pos_callback(self, msg):
		# temp position in camera_color_optical_frame frame
		temp_pose = PoseStamped()
		temp_pose.header = msg.detections[0].pose.header
		temp_pose.pose = msg.detections[0].pose.pose.pose

		#TODO: check whether the time warping is needed
		tr = self.buff.lookup_transform("map", temp_pose.header.frame_id, rospy.Time.now() - rospy.Duration(0.5), rospy.Duration(0.5))
		self.dock_position = tf2_geometry_msgs.do_transform_pose(temp_pose, tr) 
		self.temp_pub.publish(self.dock_position)



	

if __name__ == "__main__":
	x = LastMilePlanner()
	rospy.loginfo("calling m2g")
	x.move2goal()
