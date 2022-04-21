#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from math import pow, atan2, sqrt
import tf2_ros as tf
from tf2_geometry_msgs import PoseStamped

topic_vel_cmd = "cmd_vel"
topic_trilo_goal = "trilobot/goal"

class LastMilePlanner():

	def __init__(self):
		rospy.init_node("last_mile_planner", anonymous=True)
		self.pub = rospy.Publisher(topic_vel_cmd, Twist, queue_size=10)
		self.pose_sub = rospy.Subscriber(topic_trilo_goal, PoseStamped, self.update_goal)
		self.pose = PoseStamped()
		self.pose.header.frame_id = "base_link"
		self.pose.pose.position.x = 0
		self.pose.pose.position.y = 0
		self.pose.pose.position.z = 0
		self.pose.pose.orientation.x = 0
		self.pose.pose.orientation.y = 0
		self.pose.pose.orientation.z = 0
		self.pose.pose.orientation.w = 1
		
		self.buff = tf.Buffer()
		self.tfl = tf.TransformListener(self.buff)

		self.goal = PoseStamped()
		self.rate = rospy.Rate(10)
		self.rdy = False

	def update_goal(self, msg):
		#self.rdy = True
		self.goal = msg
		self.update_pos()

	def update_pos(self):
		try:
			#rospy.loginfo("trying") 
			if self.buff.can_transform("base_link", "map", rospy.Time(0)):
				#rospy.loginfo("ok")
				self.rdy = True
				self.pose = self.buff.transform(self.pose, "map", rospy.Duration(3))
		except: pass

	
	def dst(self):
		#rospy.loginfo("{}, {}".format(self.pose.header.frame_id, self.goal.header.frame_id))
		rospy.loginfo("gx: {} px: {} gy: {} py: {}".format(self.goal.pose.position.x,self.pose.pose.position.x,self.goal.pose.position.y,self.pose.pose.position.y))
		return sqrt(pow((self.goal.pose.position.x-self.pose.pose.position.x), 2) +
                    pow((self.goal.pose.position.y-self.pose.pose.position.y), 2))

			
	def move2goal(self):
		tolerance = 0.10
		while not self.rdy:
			rospy.loginfo("waiting")
		while self.dst() >= tolerance:
			#rospy.loginfo(self.dst())
			rospy.loginfo("X: " + str(self.goal.pose.position.x) + ", Y: " + str(self.goal.pose.position.y) + ", Theta: " + str(self.goal.pose.orientation.z) + ", (lin, ang): (" + str(self.linear_vel()) + str(self.angular_vel())+")")
			vel_msg = Twist()
			# Linear velocity in the x-axis.
			vel_msg.linear.x = self.linear_vel()
			vel_msg.linear.y = 0
			vel_msg.linear.z = 0
			# Angular velocity in the z-axis.
			vel_msg.angular.x = 0
			vel_msg.angular.y = 0
			vel_msg.angular.z = self.angular_vel()	
			
			self.pub.publish(vel_msg)
			self.rate.sleep()

		while self.goal.pose.orientation.z >= 0.1:
			rospy.loginfo("X: " + str(self.goal.pose.position.x) + ", Y: " + str(self.goal.pose.position.y))
			vel_msg = Twist()
			vel_msg.linear.x = 0
			vel_msg.linear.y = 0
			vel_msg.linear.z = 0
			vel_msg.angular.x = 0
			vel_msg.angular.y = 0
			vel_msg.angular.z = -0.3 if self.goal.pose.orientation.z - self.pose.pose.orientation.z > 0 else 0.3

			#self.pub.publish(vel_msg)
			self.rate.sleep()			

	def linear_vel(self):
		return 0.1*self.dst()

	def angular_vel(self):
		angle = atan2(self.goal.pose.position.y-self.pose.pose.position.y, self.goal.pose.position.x-self.pose.pose.position.x)
		return 0.1*(angle-self.pose.pose.orientation.z)		

if __name__ == "__main__":
	x = LastMilePlanner()
	rospy.loginfo("calling m2g")
	x.move2goal()
