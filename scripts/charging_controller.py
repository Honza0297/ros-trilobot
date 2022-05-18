#!/usr/bin/env python3

"""
Author: Jan Beran
Description: Charging control class. Details can be found in my diploma thesis.  
"""
# Generic includes
from math import atan2, sqrt, ceil


# ROS includes
import rospy
import actionlib

from std_msgs.msg import Empty, Bool
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import tf2_ros as tf
from tf2_geometry_msgs import PoseStamped
import tf2_geometry_msgs

from apriltag_ros.msg import AprilTagDetectionArray


#Trilobot includes
from topics import *
from trilobot.msg import Battery_state 

#################################
# Config variables
# You can try to change them in case controller does not work well
##################################
# Offset of the charging position, eg. how far "in front of" the docking station should robot go before final docking
CP_OFFSET = 0.3  # [m]
DOCK_DICOVERY_TIMEOUT = 10  # [s]
RATE = 10  # [Hz], How often to run rospy loop 

FINAL_CMD_VEL_PERIOD = 0.3  # [s], how long to send commands to ensure stable charging connection
# Officialy, it is 4.2, but voltage dividers have tendencies to error.
# And there are other ways to stop charging (both SW and HW),
# so it is not a problem to set "full voltage" a little higher.
CELL_VOLTAGE_FULL = 4.3   
FULL_CHARGING_TIME = 10800 # [s]

##################################
# Constants
##################################
STATE_INIT = 0
STATE_FETCH_DOCK = 1
STATE_CHECKING_CHARGE_NEED = 2
STATE_PREP = 3
STATE_FINAL_NAV = 4
STATE_CHARGING = 5
STATE_CHARGED = 6
STATE_FINISHED = 7




class ChargingController():
    def __init__(self) :
        rospy.init_node("charging_controller", anonymous=True)
        rospy.loginfo("Initializing Charging controller")
        self.rate = rospy.Rate(RATE)
        
        # WARNING: For debug purposes, charge needed, charge anyway and disconnect_anyway are set to true
        # to force charging sequence by default
        self.charge_needed = True # False 
        self.charge_anyway = True # False
        self.disconnect_anyway = True
        self.charging = False
        self.dock_tf_present = False
        self.cells = [-1,-1,-1,-1]
        self.state = STATE_INIT
        # How long to wait for dock to appear
        self.dock_discovery_timeout = rospy.Time.now() + rospy.Duration(DOCK_DICOVERY_TIMEOUT)

        # For transforms
        self.buff = tf.Buffer(rospy.Duration(120.0))
        self.tfl = tf.TransformListener(self.buff)

        # private velocity publisher for the final phase
        self.vel_pub = rospy.Publisher(cmd_vel, Twist, queue_size=10)
        # Suppresses main velocity controller
        self.supressor = rospy.Publisher(topic_priority_move, Bool, queue_size=10)

        self.dock_position_sub = rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.pos_callback)
        # Force charging sequence no matter how charged batteries are
        self.charge_anyway_sub = rospy.Subscriber(topic_charge_anyway, Empty, self.cac) # cac = charge anyway callback
        # Force disconnect from dock no matter how charged batteries are
        self.disconnect_anyway_sub = rospy.Subscriber(topic_disconnect_anyway, Empty, self.dac) # cac = charge anyway callback
        self.charging_checker = rospy.Subscriber(topic_charging, Bool, self.ccc) # cc = charge check callback
        self.charge_need_checker = rospy.Subscriber(topic_need_charge, Bool, self.ncc) # ncc = need charge callback
        self.voltage_checker = rospy.Subscriber(topic_battery_raw, Battery_state, self.bsc) # bsc = battery state callback

        # client for sending goals to move_base
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()


        # Robot's pose in base_link frame, used for further computations
        self.pose = PoseStamped()
        self.pose.header.frame_id = "base_link"
        self.pose.pose.position.x = 0
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = 0
        self.pose.pose.orientation.x = 0
        self.pose.pose.orientation.y = 0
        self.pose.pose.orientation.z = 0
        self.pose.pose.orientation.w = 1

        # Position in docking station when trilo is really charging
        self.goal = PoseStamped()
        self.goal.header.frame_id = "dock"
        self.goal.pose.position.x = 0
        self.goal.pose.position.y = 0
        self.goal.pose.position.z = 0.0 
        self.goal.pose.orientation.x = 0
        self.goal.pose.orientation.y = 1 
        self.goal.pose.orientation.z = 0
        self.goal.pose.orientation.w = 1

        # The same position as the previous one, but in "map" frame. The upper is constant, this is changing, but it can happen that
        # in the fnal phase, Apriltag is no longer visible by the camera and it is needed to "go blind"
        self.goal_map = PoseStamped()
        self.goal_bl = PoseStamped()
        # Charging position without time in the "map" frame AKA "in front of the dock" position
        # map frame should ensure the goal will be somehow valid even after a long time with no direct visibility
        self.charging_position = PoseStamped()    


    def pos_callback(self, msg):
        # Actually, robot does not do anything with the message - just updating my internal info about charging position.

        tr = None
        try:
            #                                dst,   src,    when                   timeout
            tr = self.buff.lookup_transform("map", "dock", rospy.Time.now(), rospy.Duration(0.5))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException)as e:
            # rospy.logwarn("Transform from dock to map frame failed: {}".format(e))
            # NOTE we should have another chance with the next iteration of the callback
            return
        # Position for final navigation in dock frame - we need to get it in map frame 
        charging_position = PoseStamped()
        charging_position.header.frame_id = "dock"
        charging_position.pose.position.x = 0
        charging_position.pose.position.y = 0
        charging_position.pose.position.z = CP_OFFSET 
        charging_position.pose.orientation.x = 0
        charging_position.pose.orientation.y = 0.707
        charging_position.pose.orientation.z = 0
        charging_position.pose.orientation.w = 0.707
        charging_position.header.stamp = rospy.Time.now()
        

        self.charging_position = tf2_geometry_msgs.do_transform_pose(charging_position, tr)
        # Tag/dock and camera are in 3D space, but Trilobot moves in 2D - so we need to set the goal in 2D.
        # Very simply, we just zero everything not important in pose orientation and normalize the quaternion. 
        self.charging_position.pose.orientation.x = 0
        self.charging_position.pose.orientation.y = 0
        q = (self.charging_position.pose.orientation.x, self.charging_position.pose.orientation.y, self.charging_position.pose.orientation.z, self.charging_position.pose.orientation.w)
        m = sum( i*i for i in q)
        if abs(m-1.0) > 0.0001:
            ms = sqrt(m)
            q = tuple(i/ms for i in q)
        self.charging_position.pose.orientation.x = q[0]
        self.charging_position.pose.orientation.y = q[1]
        self.charging_position.pose.orientation.z = q[2]
        self.charging_position.pose.orientation.w = q[3]

        self.dock_tf_present = True

     # Battery state callback   
    def bsc(self, msg):
        self.cells[0] = msg.cell1
        self.cells[1] = msg.cell2
        self.cells[2] = msg.cell3
        self.cells[3] = msg.cell4

    # Disconnect anyway callback
    def dac(self, msg):
        self.disconnect_anyway = msg.data

    # Need charge callback
    def ncc(self, msg):
        if not self.charge_anyway:
            self.charge_needed = msg.data

    # Charge anyway callback
    def cac(self, msg):
        self.charge_anyway = True
        self.charge_needed = True

    # Check charge callback
    def ccc(self, msg):
        self.charging = msg.data

    # Main method where state transitions take place
    def update(self):
        while not rospy.is_shutdown():
            if self.state == STATE_INIT:
                rospy.loginfo("INIT")
                self.state = STATE_FETCH_DOCK
            elif self.state == STATE_FETCH_DOCK:
                rospy.loginfo("FETCH_DOCK")
                self.fetch_dock()
            elif self.state == STATE_CHECKING_CHARGE_NEED:
                rospy.loginfo("CHECK_CHARGE_NEED")
                self.check_charge_need()
            elif self.state == STATE_PREP:
                rospy.loginfo("PREP")
                self.prepare_for_charge_goal()
            elif self.state == STATE_FINAL_NAV:
                rospy.loginfo("FINAL APPROACHING")
                self.final_approaching()
            elif self.state == STATE_CHARGING:
                rospy.loginfo("CHARGING")
                self.check_charging()
            elif self.state == STATE_CHARGED:
                rospy.loginfo("CHARGED")
                self.disconnect()
            elif self.state == STATE_FINISHED:
                rospy.loginfo("FINISHED")
                self.finish()
            self.rate.sleep()

    def fetch_dock(self):
        rospy.loginfo("Fetching the dock position")
        # The actual work about setting charging pose is in pos_callback()
        next_state = STATE_FETCH_DOCK
        if self.dock_tf_present:
            next_state = STATE_CHECKING_CHARGE_NEED
        self.state = next_state

    def check_charge_need(self):
        rospy.loginfo("Checking the need of charging")
        # The actual work about checking the charge of need is done in ncc() and cac()
        if self.charge_needed:
            # self.state = STATE_PREP
            self.state = STATE_FINAL_NAV

    def prepare_for_charge_goal(self):
        rospy.loginfo("Preparing for charging")

        rospy.loginfo("Supressing standard cmd_vel pipeline")
        # There was an idea to have separated move bridges for standard and prioritized move commands
        # However, the idea is on half way: move bridge supports priority modes, but still does
        # not actually suppress the "standard" commands, just warns about prioritized move in progress
        # The reason is that we cannot dynamically change topic remap in move_base (eg. from cmd_vel to cmd_vel/prioritized
        # or something like that).
        # FIXME ^^ - But probably after diploma thesis.
        msg = Bool()
        msg.data = True
        self.supressor.publish(msg) # pub info about prioritized move

        rospy.loginfo("Sending a goal to move_base")
        # the goal is "in front of the dock" position
        goal = MoveBaseGoal()
        self.charging_position.header.stamp = rospy.Time.now() # Need to pretend the goal is fresh
        goal.target_pose = self.charging_position
        self.client.send_goal(goal)
        res = None
        wait = self.client.wait_for_result()
        if not wait: # Assuming action server is down
            rospy.logerr("Action server not available! Wait is: {}, res is: {}.".format(wait, self.client.get_result()))
            rospy.signal_shutdown("Action server not available!")
        else:
            res = self.client.get_result()
            rospy.loginfo("Result from move_base: {}".format(res))
        self.state = STATE_FINAL_NAV

    def wiggle(self):
        angular = 0.1
        num_of_wiggles = 1
        msg = Twist()
        for i in range(num_of_wiggles):
            msg.angular.z = angular
            self.vel_pub.publish(msg)
            rospy.sleep(rospy.Duration(0.1))
            msg.angular.z = -angular
            self.vel_pub.publish(msg)
            rospy.sleep(rospy.Duration(0.1))


    def final_approaching(self):
        #rospy.info("Now, i should perform final navigation to the station, not sure whether I am capable to do it :(")
        rospy.loginfo("Final navigation to the dock.")
        start = rospy.Time.now()
        # Navigation not with move_base node, but "manually"
        while not self.charging:
            self.move2goal()
            if rospy.Time.now() - start > rospy.Duration(5): # more than 5 seconds of final nav
                self.wiggle()
                start = rospy.Time.now()
            self.rate.sleep()

        # Ensure contact is stable - once charging, go a little further to push contacts
        cnt = ceil(FINAL_CMD_VEL_PERIOD / (1/RATE)) # set how many commands send 
        msg = Twist()
        msg.linear.x = 0.01 # Internally, Trilobot will go forward with the smallest possible speed
        while cnt:
            self.vel_pub.publish(msg)
            cnt -= 1
            self.rate.sleep()
        self.state = STATE_CHARGING


    def check_charging(self):
        rospy.loginfo("Charging, setting the time needed to charge...")

        # Currently, time needed for charging is computed very roughly from battery voltages. 
        # Assuming quite a big error, but that should not be a problem.
        # Overall charging time from 0 to full with 2A current: 3 hours (CC/CV) or 10800 sec

        # Linear regression of votlage dependence on capacity, 4.2 ~ full, 3.4 ~ 0.1 capacity left
        v2c = lambda v: 1.125*v-3.725
        # Average remaining capacity (ignoring negative values caused by the aproximation above)
        remaining_capacity = sum([v2c(cell) if v2c(cell) > 0 else 0 for cell in self.cells])/len(self.cells)

        charging_time = rospy.Duration(FULL_CHARGING_TIME*(1-remaining_capacity))
        rospy.loginfo("Charging time will be {} seconds.".format(charging_time))
        charging_start = rospy.Time.now()
        while rospy.Time.now()-charging_start < charging_time or max(self.cells) < CELL_VOLTAGE_FULL or not self.disconnect_anyway: 
            self.rate.sleep()
        
        self.state = STATE_CHARGED

    def disconnect(self):
        rospy.loginfo("Trilobot charged, disconnecting...")
        # Currently, "disconnect" means really just go backwards for a while
        cnt = ceil(1 / (1 / RATE))  # set how much commands send, the aim is to send commands for 1 s
        msg = Twist()
        msg.linear.x = -0.01  # Internally, Trilobot will go backward with the smallest possible speed
        while cnt or self.charging:
            self.vel_pub.publish(msg)
            cnt -= 1
            self.rate.sleep()
        self.state = STATE_FINISHED

    def finish(self):
        rospy.loginfo("Finishing...")
        rospy.loginfo("Reset prioritized move flag in move_bridge")
        msg = Bool()
        msg.data = False
        self.supressor.publish(msg)
        self.charge_anyway = False
        self.charge_needed = False
        self.disconnect_anyway = False
        # self.state = STATE_CHECKING_CHARGE_NEED
        rospy.loginfo("Charging finished successfully!")

        #rospy.logfatal("Task failed successfully! :) ")
        #rospy.signal_shutdown("Task failed successfully! :) ")
    
    ##################################################
    # Methods for manual docking and navigation
    ##################################################

    def dst(self, src, dst):
        if src == None or dst == None:
            return 1000  # [km] one kilometer should be above any possible tolerance :)
        return sqrt(pow((dst.pose.position.x - src.pose.position.x), 2) +
                    pow((dst.pose.position.y - src.pose.position.y), 2))

    def linear_vel(self, src, dst):
        vel = self.dst(src, dst)
        return vel 

    def angular_vel(self, src, dst):
        dy = dst.pose.position.y - src.pose.position.y
        dx = dst.pose.position.x - src.pose.position.x
        angle = atan2(dy, dx)
        avel = angle - src.pose.orientation.z
        return avel

    def move2goal(self):
        pose_map = PoseStamped()
        #goal_map = PoseStamped()
        goal_tr = None
        pose_tr = None
        self.pose.header.stamp = rospy.Time.now()
        self.goal.header.stamp = rospy.Time.now()
        ok = True
        try:
            pose_tr = self.buff.lookup_transform("map", self.pose.header.frame_id, rospy.Time.now(),
                                                 rospy.Duration(0.1))
            pose_map = tf2_geometry_msgs.do_transform_pose(self.pose, pose_tr)
        except (tf.LookupException, tf.ExtrapolationException) as e:
            ok = False
            rospy.logwarn(e)


        try:  # dock in bl
            goal_tr = self.buff.lookup_transform("base_link", self.goal.header.frame_id,
                                                 rospy.Time(), rospy.Duration(0.1))
            self.goal_bl = tf2_geometry_msgs.do_transform_pose(self.goal, goal_tr)

        except (tf.LookupException, tf.ExtrapolationException) as e:
            if self.dock_tf_present:
                rospy.logwarn("Cannot update goal position in base_link frame - using the old one...")
            else:
                return

            #ok = False
            #rospy.logwarn("An error occured when converting goal pose to the map frame: {}".format(e))
        retval = (float("inf"), float("inf"))
        if ok:
            #if self.dst(pose_map,
            #            self.goal_map) <= tolerance and self.goal_map.pose.orientation.z - pose_map.pose.orientation.z < tolerance_theta:
            #rospy.loginfo("Goal achieved.")
            # TODO set flag?
            #rospy.loginfo(
            #    "dX: {}, dY: {}, dTheta: {}".format(goal_map.pose.position.x - pose_map.pose.position.x,
            #                                        goal_map.pose.position.y - pose_map.pose.position.y,
            #                                        goal_map.pose.orientation.z - pose_map.pose.orientation.z))
            #return
            x = self.linear_vel(self.pose, self.goal_bl)
            z = self.angular_vel(self.pose, self.goal_bl)*1.4
            a = 1
            vel_max = 0.1
            if x > vel_max:
                a = x / vel_max
                x = vel_max
                z = z / a * 1.5
            vel_msg = Twist()
            # Linear velocity in the x-axis.
            vel_msg.linear.x = x
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = z
            #rospy.logwarn("X:{}, Z:{}\n".format(vel_msg.linear.x, vel_msg.angular.z))
            self.vel_pub.publish(vel_msg)
            retval = (x, z)
        return retval
            #self.rate.sleep()




if __name__ == "__main__":
    cc = ChargingController()
    rospy.loginfo("Starting Charging controller")
    cc.update()
    #rospy.spin()
