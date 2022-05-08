#!/usr/bin/env python3
from math import atan2, sqrt, ceil

import rospy
from trilobot.msg import Battery_state 
from std_msgs.msg import Empty, String, Int32, Bool
from sensor_msgs.msg import BatteryState
from topics import *
from geometry_msgs.msg import Twist

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import tf2_ros as tf
from tf2_geometry_msgs import PoseStamped
import tf2_geometry_msgs

from apriltag_ros.msg import AprilTagDetectionArray


# Offset of the charging position, eg. how far "in front of" the docking station should robot go before final docking
CP_OFFSET = 0.3  # [m]
DOCK_DICOVERY_TIMEOUT = 10  # [s]
RATE = 10  # [Hz]
FINAL_CMD_VEL_PERIOD = 0.3  # [s]
CELL_VOLTAGE_FULL = 4.2

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

         # current robot position in base link
        # USE:
        self.pose = PoseStamped()
        self.pose.header.frame_id = "base_link"
        self.pose.pose.position.x = 0
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = 0
        self.pose.pose.orientation.x = 0
        self.pose.pose.orientation.y = 0
        self.pose.pose.orientation.z = 0
        self.pose.pose.orientation.w = 1

        # Position in docking station when trilo is really charging (not the "in front of" pose!)
        self.goal = PoseStamped()
        self.goal.header.frame_id = "dock"
        self.goal.pose.position.x = 0
        self.goal.pose.position.y = 0
        self.goal.pose.position.z = 0.0 # TODO WARNING Nastavit to tak, aby trilo zadokoval, ale zase dokynu nepřejel!
        self.goal.pose.orientation.x = 0
        self.goal.pose.orientation.y = 1 # TODO is this correct
        self.goal.pose.orientation.z = 0
        self.goal.pose.orientation.w = 1

        # Charging position without time in the "map" frame AKA "in front of the dock" position
        # map frame should ensure the goal will be somehow valid even after a long time - and not direct visibility
        self.charging_position = PoseStamped()    

        # WARNING: For debug purposes, charge needed and charge anyway are set to true
        # to force charging sequence by default
        self.charge_needed = True # False 
        self.charge_anyway = True # False
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
        self.dock_position_sub = rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.pos_callback)
        # Suppresses main velocity controller
        self.supressor = rospy.Publisher(topic_priority_move, Bool, queue_size=10)
        # Force charging sequence no matter how charged batteries are
        self.charge_anyway_sub = rospy.Subscriber(topic_charge_anyway, Empty, self.cac) # cac = charge anyway callback
        self.charging_checker = rospy.Subscriber(topic_charging, Bool, self.ccc) # cc = charge check callback
        self.charge_need_checker = rospy.Subscriber(topic_need_charge, Bool, self.ncc) # ncc = need charge callback
        self.voltage_checker = rospy.Subscriber(topic_battery_raw, Battery_state, self.bsc) # bsc = battery state callback
        # NOTE: delete once done, server as a debug
        self.temp_pub = rospy.Publisher("temp_dock_pose", PoseStamped, queue_size=10)
        self.temp_goal_pub = rospy.Publisher("trilobot/goal", PoseStamped, queue_size=10)

        # client for sending goals to move_base
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()


       

    def bsc(self, msg):
        self.cells[0] = msg.cell1
        self.cells[1] = msg.cell2
        self.cells[2] = msg.cell3
        self.cells[3] = msg.cell4


    def pos_callback(self, msg):
        # Actually, I dont do anything with the message - just updating my internal info about charging position.
        # Check whether the initialization is finished. Currently another
        # approach is tested. TODO If no Attribute Error, delete this
        #try:
        #    dummy = self.buff
        #except AttributeError as e:
        #    rospy.logwarn("Buffer not available for now: {}".format(e))
        #    return
        tr = None
        try:
            # TODO: check whether the time warping is needed
            # the line should be like this: tr = self.buff.lookup_transform("map", "dock", rospy.Time.now(), rospy.Duration(0.5))
            #                                dst,   src,                when                            timeout
            tr = self.buff.lookup_transform("map", "dock", rospy.Time.now(), rospy.Duration(0.5))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException)as e:
            #rospy.logerr("Transform from dock to map failed: {}".format(e))
            # NOTE we should have another chance with the next iteration of the callback
            return
        charging_position = PoseStamped()
        charging_position.header.frame_id = "dock"
        charging_position.pose.position.x = 0
        charging_position.pose.position.y = 0
        charging_position.pose.position.z = CP_OFFSET # TODO check whether the offset is correct
        charging_position.pose.orientation.x = 0
        charging_position.pose.orientation.y = 0.707
        charging_position.pose.orientation.z = 0
        charging_position.pose.orientation.w = 0.707
        charging_position.header.stamp = rospy.Time.now()
        self.charging_position = tf2_geometry_msgs.do_transform_pose(charging_position, tr)
        self.dock_tf_present = True
        self.temp_pub.publish(self.charging_position)

    # TODO reset flags after charging is done!
    def ncc(self, msg):
        if not self.charge_anyway:
            self.charge_needed = msg.data

    def cac(self, msg):
        self.charge_anyway = True
        self.charge_needed = True

    def ccc(self, msg):
        self.charging = msg.data

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
                #rospy.signal_shutdown("First test complete (stoped just before execution of state_prep)!")
                #return
                # here I should be several seconds - is it a problem?
                # TODO check ^^
                rospy.loginfo("PREP2")
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
        # the actual work about checking the charge of need is done in ncc() and cac()
        if self.charge_needed:
            self.state = STATE_PREP

    def prepare_for_charge_goal(self):
        rospy.loginfo("Preparing for charging")

        rospy.loginfo("Supressing standard cmd_vel pipeline")
        # There was an idea to have separated move bridges for standard and prioritized move commands
        # However, the idea is on half way: move bridge supports priority modes, but still does
        # not actually suppress the "standard" commands, just warns about prioritized move in progress
        # The reason is that we cannot dynamically change topic remap in move_base (eg. from cmd_vel to cmd_vel/prioritized
        # or something like that).
        # FIXME TODO ^^
        msg = Bool()
        msg.data = True
        self.supressor.publish(msg) # pub info about prioritized move

        rospy.loginfo("Cancelling current goal in move_base if there is any")
        # TODO cancel move base goal if there is any

        rospy.loginfo("Sending a goal to move_base")
        # the goal is "in front of the dock" position
        goal = MoveBaseGoal()

        # magic to get the goal to planar - could be done in the pose_callback as well
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


        self.charging_position.header.stamp = rospy.Time.now()
        self.temp_goal_pub.publish(self.charging_position)
        goal.target_pose = self.charging_position
        self.client.send_goal(goal)
        res = None
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available! Wait is: {}, res is: {}.".format(wait, self.client.get_result()))
            rospy.signal_shutdown("Action server not available!")
        else:
            res = self.client.get_result()
            rospy.loginfo("Result from move_base: {}, wait: {}".format(res, wait))
        #rospy.spin() # ??
        self.state = STATE_FINAL_NAV

    def final_approaching(self):
        rospy.loginfo("Now, i should perform final navigation to the station, not sure whether I am capable to do it :(")
        # not with move base, but "manually"
        while not self.charging:
            self.move2goal()
            self.rate.sleep()

        cnt = ceil(FINAL_CMD_VEL_PERIOD / (1/RATE)) # set how many commands send - aim si to send commands for 0.3 s +-
        msg = Twist()
        msg.linear.x = 0.01 # Internally, Trilobot will go forward with the smallest possible speed
        # now charging - popojedu ještě trochu, aby byla jistotka
        while cnt:
            self.vel_pub.publish(msg)
            cnt -= 1
            self.rate.sleep()
        self.state = STATE_CHARGING

    def check_charging(self):
        rospy.loginfo("Charging, checking the voltage levels...")
        if max(self.cells) > CELL_VOLTAGE_FULL:
            self.state = STATE_CHARGED

    def disconnect(self):
        rospy.loginfo("Trilobot charged, disconnecting...")
        cnt = ceil(1 / (1 / RATE))  # set how much commands send, the aim is to send commands for 1 s
        msg = Twist()
        msg.linear.x = -0.01  # Internally, Trilobot will go backward with the smallest possible speed
        while cnt or self.charging:
            self.vel_pub.publish(msg)
            cnt -= 1
            self.rate.sleep()
        self.charge_anyway = False
        self.charge_needed = False
        self.state = STATE_FINISHED

    def finish(self):
        rospy.loginfo("Finishing...")
        rospy.loginfo("Reset prioritized move flag in move_bridge")
        msg = Bool()
        msg.data = False
        self.supressor.publish(msg)

        rospy.logfatal("Task failed successfully! :) ")
        rospy.signal_shutdown("Task failed successfully! :) ")

    def dst(self, src, dst):
        if src == None or dst == None:
            return 1000  # one kilometer should be above any possible tolerance :)
        return sqrt(pow((dst.pose.position.x - src.pose.position.x), 2) +
                    pow((dst.pose.position.y - src.pose.position.y), 2))

    def linear_vel(self, src, dst):
        vel = self.dst(src, dst)
        return vel  # if vel > 0.05 else 0.05

    def angular_vel(self, src, dst):
        angle = atan2(dst.pose.position.y - src.pose.position.y, dst.pose.position.x - src.pose.position.x)
        avel = angle - src.pose.orientation.z
        return avel  # if avel > 0.05 else 0.05

    def move2goal(self):
        tolerance = 0.1 # in x,y
        tolerance_theta = 0.1 # theta
        pose_map = PoseStamped()
        goal_map = PoseStamped()
        goal_tr = None
        pose_tr = None

        #rospy.loginfo("{}".format(self.dst(pose_map, goal_map)))
        self.pose.header.stamp = rospy.Time.now()
        self.goal.header.stamp = rospy.Time.now()
        ok = True
        try:
            pose_tr = self.buff.lookup_transform("map", self.pose.header.frame_id, rospy.Time.now(),
                                                 rospy.Duration(0.2))
            pose_map = tf2_geometry_msgs.do_transform_pose(self.pose, pose_tr)
        except (tf.LookupException, tf.ExtrapolationException) as e:
            ok = False
            rospy.logwarn(e)

        try:  # dock
            goal_tr = self.buff.lookup_transform("map", self.goal.header.frame_id,
                                                 rospy.Time.now(), rospy.Duration(0.5))
            goal_map = tf2_geometry_msgs.do_transform_pose(self.goal, goal_tr)
        except (tf.LookupException, tf.ExtrapolationException) as e:
            ok = False
            rospy.logwarn(e)
        if ok:
            if self.dst(pose_map,
                        goal_map) <= tolerance and goal_map.pose.orientation.z - pose_map.pose.orientation.z < tolerance_theta:
                rospy.loginfo("Goal achieved.")
                # TODO set flag?
                #rospy.loginfo(
                #    "dX: {}, dY: {}, dTheta: {}".format(goal_map.pose.position.x - pose_map.pose.position.x,
                #                                        goal_map.pose.position.y - pose_map.pose.position.y,
                #                                        goal_map.pose.orientation.z - pose_map.pose.orientation.z))
                return
            else:
                x = self.linear_vel(pose_map, goal_map)
                z = self.angular_vel(pose_map, goal_map)
                a = 1
                if x > 0.1:
                    a = x / 0.1
                    x = 0.1
                    z = z / a * 1.1
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

            #self.rate.sleep()




if __name__ == "__main__":
    cc = ChargingController()
    rospy.loginfo("Starting Charging controller")
    cc.update()
    #rospy.spin()