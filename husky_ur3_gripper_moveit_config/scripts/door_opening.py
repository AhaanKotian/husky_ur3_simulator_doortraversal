#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
import copy, math
import tf
from math import pi, radians, degrees, atan2, sqrt
from moveit_commander import MoveGroupCommander, RobotCommander 
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Twist
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, MoveItErrorCodes, DisplayTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import random
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import String
from nav_msgs.msg import Path
from robot_to_door_handle_msg.msg import Message
from trac_ik_python.trac_ik import IK


#GROUP_NAME_GRIPPER = "NAME OF GRIPPER"
roscpp_initialize(sys.argv)
rospy.init_node('door_opening_s2', anonymous=True)
robot = RobotCommander()
scene = PlanningSceneInterface()

ik_solver = IK("ur3_base_link","ee_link")
seed_state = [0.0] * ik_solver.number_of_joints
goal_state = [0.0] * ik_solver.number_of_joints


##모바일 파트 관련 변수
x = 0.0
y = 0.0 
theta = 0.0

group_name = "ur3_manipulator"
move_group = MoveGroupCommander(group_name)
FIXED_FRAME = 'map'

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               DisplayTrajectory,
                                               queue_size=20)
move_group.go([1.57,-2.27,1.93,-1.19,-1.57,0], wait=True)

listener = tf.TransformListener()


msg = rospy.wait_for_message("/door_planner/door_handle_path", Message, timeout=2000)
i=0
joint_waypoints = []
for pose_i in msg.door_handle_pose.poses:
    if pose_i.pose.position.y < -0.90:
        continue
    
    listener.waitForTransform("/map","ur3_base_link",rospy.Time.now(),rospy.Duration(10.0))
    door_handle_pose_transformed = listener.transformPose("ur3_base_link",pose_i)
    
    # goal_state = ik_solver.get_ik(seed_state,
    #             door_handle_pose_transformed.pose.position.x, door_handle_pose_transformed.pose.position.y, door_handle_pose_transformed.pose.position.z,
    #             door_handle_pose_transformed.pose.orientation.x, door_handle_pose_transformed.pose.orientation.y, door_handle_pose_transformed.pose.orientation.z, door_handle_pose_transformed.pose.orientation.w)
    print("msg.robot_pose.poses[i] = ",[msg.robot_pose.poses[i].pose.position.x,msg.robot_pose.poses[i].pose.position.y])
    print("door_handle_pose_transformed = ",[door_handle_pose_transformed.pose.position.x,door_handle_pose_transformed.pose.position.y])
    joint_waypoints.append([msg.robot_pose.poses[i],door_handle_pose_transformed])
    i = i+1

def amcl_pose_callback(msg) :
    # (x,y) = (msg.pose.pose.position.x,msg.pose.pose.position.y)
    print("In amcl_pose_callback()")
    # for i in joint_waypoints:
    #     if(math.dist([i[0].pose.position.x,i[0].pose.position.y,i[0].pose.position.z],[msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z]) <= 0.1 ) :
    #         print("i[1] = ",i[1])
    #         move_group.go(i[1],wait=True)
    k=0
    for i in joint_waypoints:
        if(math.dist([i[0].pose.position.x,i[0].pose.position.y,i[0].pose.position.z],[msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z]) <= 0.1 ) :
            
            goal_state = ik_solver.get_ik(seed_state,
                i[1].pose.position.x, i[1].pose.position.y, i[1].pose.position.z,
                i[1].pose.orientation.x, i[1].pose.orientation.y, i[1].pose.orientation.z, i[1].pose.orientation.w)
            print("goal_state",goal_state)
            move_group.go(goal_state,wait=True)

while not rospy.is_shutdown():
    sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, amcl_pose_callback)
    



 