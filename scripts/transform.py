#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import  PoseStamped, PoseWithCovariance
from apriltag_ros.msg import AprilTagDetectionArray 
from tf2_msgs.msg import TFMessage
topic_goal = "/rtabmap/goal"

topic_tags = "/tag_detections"

pose = PoseStamped()

def callback(msg):
    #rospy.loginfo(msg.detections[0].pose)
    if not msg.detections:
        return
    pose.header = msg.detections[0].pose.header
    pose.header.frame_id = "dock"
    
    #pose.pose = msg.detections[0].pose.pose.pose
    
    pose.pose.position.z = 0.25
    #pose.pose.position.x = 0
    #pose.pose.position.y = 0
    pose.pose.orientation.y = 1
    pose.pose.orientation.w = 1

if __name__ == "__main__":
    rospy.init_node("trilo_transformer")
    tfl = tf.TransformListener()
    sub = rospy.Subscriber(topic_tags, AprilTagDetectionArray, callback)
    pub = rospy.Publisher(topic_goal, PoseStamped, queue_size=1, latch=True)
    
   
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        #rospy.loginfo(pose)
        if pose.header.frame_id and tfl.frameExists("camera_color_optical_frame") and tfl.frameExists("dock") and tfl.canTransform("dock","camera_color_optical_frame", rospy.Time()):
            tfl.waitForTransform("dock","camera_color_optical_frame", rospy.Time.now(), rospy.Duration(4.0))
            pub.publish(tfl.transformPose("camera_color_optical_frame", pose))
        rate.sleep()





