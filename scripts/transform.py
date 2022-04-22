#!/usr/bin/env python3

import rospy
import tf2_ros as tf
from geometry_msgs.msg import  PoseStamped, PoseWithCovariance
from apriltag_ros.msg import AprilTagDetectionArray 
from tf2_msgs.msg import TFMessage
import tf2_geometry_msgs as tf2_msg
topic_goal = "/rtabmap/goal" # TODO pozdjei zmenit na move_base_simple/goal - az bude fungovat mapovani
topic_goal_trilo = "/trilobot/goal" # goal translated to coordinate frame of the trilobot
topic_tags = "/tag_detections"

pose = tf2_msg.PoseStamped()

def callback(msg):
    #rospy.loginfo(msg.detections[0].pose)
    if not msg.detections:
        return
    pose.header = msg.detections[0].pose.header
    pose.header.frame_id = "dock"
    
    pose.pose.position.z = 0.3
    #pose.pose.position.x = 0
    #pose.pose.position.y = 0
    pose.pose.orientation.y = 1
    pose.pose.orientation.w = 1

if __name__ == "__main__":
    rospy.init_node("trilo_transformer")
    buff = tf.Buffer()
    tfl = tf.TransformListener(buff)
    

    #sub = rospy.Subscriber(topic_tags, AprilTagDetectionArray, callback)
    pub_goal = rospy.Publisher(topic_goal, tf2_msg.PoseStamped, queue_size=1, latch=True)
    pub_trilo = rospy.Publisher(topic_goal_trilo,tf2_msg.PoseStamped, queue_size=1, latch=True)    
   
    # 30 cm in front of tag
    pose.header.frame_id = "dock"
    pose.pose.position.z = 0.3
    #pose.pose.position.x = 0
    #pose.pose.position.y = 0
    pose.pose.orientation.y = 0.5
    pose.pose.orientation.w = 0.5

    out_pose_camera = tf2_msg.PoseStamped()
    out_pose_trilo = tf2_msg.PoseStamped()
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        rospy.loginfo("Transforming from dock tf to camera and base_link frames")
        pose.header.stamp = rospy.Time(0)
        
        try:
            if buff.can_transform("dock", "camera_color_optical_frame", rospy.Time(0)):
                out_pose_camera = buff.transform(pose, "camera_color_optical_frame", rospy.Duration(3))
            if buff.can_transform("dock", "map", rospy.Time(0)):
                out_pose_trilo = buff.transform(pose, "map", rospy.Duration(3))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            raise
        pub_trilo.publish(out_pose_trilo)
        pub_goal.publish(out_pose_camera)
        rate.sleep()
