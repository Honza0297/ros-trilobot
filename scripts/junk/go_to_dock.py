#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from math import pow, atan2, sqrt
import tf2_ros as tf
from tf2_geometry_msgs import PoseStamped
import tf2_geometry_msgs

topic_vel_cmd = "cmd_vel"
topic_trilo_goal = "trilobot/goal"

class LastMilePlanner():

	def __init__(self):
		rospy.init_node("last_mile_planner", anonymous=True)
		self.pub = rospy.Publisher(topic_vel_cmd, Twist, queue_size=10)
		#self.pose_sub = rospy.Subscriber(topic_trilo_goal, PoseStamped, self.update_goal)
		self.pose = PoseStamped()
		self.pose.header.frame_id = "base_link"
		self.pose.pose.position.x = 0
		self.pose.pose.position.y = 0
		self.pose.pose.position.z = 0
		self.pose.pose.orientation.x = 0
		self.pose.pose.orientation.y = 0
		self.pose.pose.orientation.z = 0
		self.pose.pose.orientation.w = 1
		
		self.goal = PoseStamped()
		self.goal.header.frame_id = "dock"
		self.goal.pose.position.x = 0
		self.goal.pose.position.y = 0
		self.goal.pose.position.z = 0.1
		self.goal.pose.orientation.x = 0
		self.goal.pose.orientation.y = 0
		self.goal.pose.orientation.z = 1
		self.goal.pose.orientation.w = 1
		self.buff = tf.Buffer(rospy.Duration(120.0))
		self.tfl = tf.TransformListener(self.buff)

		self.rate = rospy.Rate(10)

	
	def dst(self, src, dst):
		if src == None or dst == None:
			return 1000 # one kilometer should be above any possible tolerance :)
		return sqrt(pow((dst.pose.position.x-src.pose.position.x), 2) +
                    pow((dst.pose.position.y-src.pose.position.y), 2))
	
	def linear_vel(self, src, dst):
		vel = self.dst(src, dst)
		return vel #if vel > 0.05 else 0.05

	def angular_vel(self,src, dst):
		angle = atan2(dst.pose.position.y-src.pose.position.y, dst.pose.position.x-src.pose.position.x)
		avel = angle-src.pose.orientation.z
		return avel #if avel > 0.05 else 0.05		
			
	def move2goal(self):
		tolerance = 0.7
		pose_map = PoseStamped()
		goal_map = PoseStamped()
		goal_tr = None
		pose_tr = None
		while not rospy.is_shutdown():
			rospy.loginfo("{}".format(self.dst(pose_map, goal_map)))
			self.pose.header.stamp = rospy.Time.now()
			self.goal.header.stamp = rospy.Time.now()
			ok = True
			try:
				pose_tr = self.buff.lookup_transform("map", self.pose.header.frame_id, rospy.Time.now(), rospy.Duration(0.2))
				pose_map = tf2_geometry_msgs.do_transform_pose(self.pose, pose_tr) 
			except (tf.LookupException, tf.ExtrapolationException) as e:
				ok = False
				rospy.logwarn(e) 
			
			try: # dock
				goal_tr = self.buff.lookup_transform("map", self.goal.header.frame_id, rospy.Time.now() - rospy.Duration(0.5), rospy.Duration(0.5))
				goal_map = tf2_geometry_msgs.do_transform_pose(self.goal, goal_tr) 
			except (tf.LookupException, tf.ExtrapolationException) as e:
				ok = False
				rospy.logwarn(e)
			if ok:
				if self.dst(pose_map, goal_map) <= tolerance and goal_map.pose.orientation.z - pose_map.pose.orientation.z < 0.1:
					rospy.loginfo("Goal achieved.")
					rospy.loginfo("dX: {}, dY: {}, dTheta: {}".format(goal_map.pose.position.x - pose_map.pose.position.x, goal_map.pose.position.y - pose_map.pose.position.y, goal_map.pose.orientation.z - pose_map.pose.orientation.z))
					break
				else:
					x = self.linear_vel(pose_map, goal_map)
					z = self.angular_vel(pose_map, goal_map)
					a = 1
					if x > 0.1:
						a = x/0.1
						x = 0.1
						z = z/a*1.1
					vel_msg = Twist()
					# Linear velocity in the x-axis.
					vel_msg.linear.x = x
					vel_msg.linear.y = 0
					vel_msg.linear.z = 0
					# Angular velocity in the z-axis.
					vel_msg.angular.x = 0
					vel_msg.angular.y = 0
					vel_msg.angular.z = z	
					rospy.logwarn("X:{}, Z:{}\n".format(vel_msg.linear.x, vel_msg.angular.z))
					self.pub.publish(vel_msg)
			
				
			self.rate.sleep()			

	

if __name__ == "__main__":
	x = LastMilePlanner()
	rospy.loginfo("calling m2g")
	x.move2goal()
