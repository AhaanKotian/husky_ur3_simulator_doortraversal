#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
import copy, math
import tf
import tf2_ros
import threading
from math import pi, radians, degrees, atan2, sqrt
from moveit_commander import MoveGroupCommander, RobotCommander 
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from moveit_commander.conversions import pose_to_list
# from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Twist
from tf2_geometry_msgs import PointStamped
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, MoveItErrorCodes, DisplayTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import random
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import String
from nav_msgs.msg import Path, Odometry
from nav_msgs.srv import GetPlan
from robot_to_door_handle_msg.msg import Message
from trac_ik_python.trac_ik import IK
roscpp_initialize(sys.argv)
rospy.init_node('test_door_opening_s2', anonymous=True)

# test = PointStamped()
# test.header.stamp = rospy.Time(0)
# test.header.frame_id = "map"
# test.point.x = 0.563691
# test.point.y = -0.471846
# test.point.z = 0.5
# tfBuffer = tf2_ros.Buffer()
# listener = tf2_ros.TransformListener(tfBuffer)
# test_transformed = tfBuffer.transform(test,"ur3_base_link",timeout=rospy.Duration(10.0))
# print(" test_transformed(x,y,z) = ({},{},{})".format(test_transformed.point.x, test_transformed.point.y, test_transformed.point.z))

test = PointStamped()
test.header.stamp = rospy.Time(0)
test.header.frame_id = "ur3_base_link"
test.point.x = float(input("x = "))
test.point.y = float(input("y = "))
test.point.z = float(input("z = "))
# tfBuffer = tf2_ros.Buffer()
# listener = tf2_ros.TransformListener(tfBuffer)
# test_transformed = tfBuffer.transform(test,"ur3_base_link",timeout=rospy.Duration(10.0))
# print(" test_transformed(x,y,z) = ({},{},{})".format(test_transformed.point.x, test_transformed.point.y, test_transformed.point.z))

ik_solver = IK("ur3_base_link","ee_link")
seed_state = [0.0] * ik_solver.number_of_joints
goal_state = [0.0] * ik_solver.number_of_joints


group_name = "ur3_manipulator"
move_group = MoveGroupCommander(group_name)
FIXED_FRAME = 'map'

goal_state, seed_state
goal_state = ik_solver.get_ik(seed_state,
            test.point.x, test.point.y, test.point.z,
            0.0, 0.0, 0.0, 1,0)

# print(" base_pose_i(x,y) = ({},{}) & door_handle_pose_transformed(x,y,z) = ({},{},{})".format(base_pose_i.pose.position.x,base_pose_i.pose.position.y, door_handle_ps_transformed.point.x, door_handle_ps_transformed.point.y, door_handle_ps_transformed.point.z))
print("goal_state",goal_state)
move_group.go(goal_state,wait=True)

# test = PoseStamped()
# test.header.stamp = rospy.Time(0)
# test.header.frame_id = "map"
# test.pose.position.x = 0.563691
# test.pose.position.y = -0.471846
# test.pose.position.z = 0.5
# test.pose.orientation.x = 0.0
# test.pose.orientation.y = 0.0
# test.pose.orientation.z = 0.0
# test.pose.orientation.w = 1.0
# listener = tf.TransformListener()
# listener.waitForTransform("/map","ur3_base_link",rospy.Time.now(),rospy.Duration(10.0))
# door_handle_pose_transformed = listener.transformPose("ur3_base_link",test)
# print(" test(x,y,z,r,p,y) = ({},{},{},{},{},{})".format(test.pose.position.x, test.pose.position.y, test.pose.position.z, test.pose.orientation.x, test.pose.orientation.y, test.pose.orientation.z))