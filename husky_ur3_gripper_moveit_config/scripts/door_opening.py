#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
import copy, math
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


#GROUP_NAME_GRIPPER = "NAME OF GRIPPER"
roscpp_initialize(sys.argv)
rospy.init_node('door_opening_s2', anonymous=True)
robot = RobotCommander()
scene = PlanningSceneInterface()



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

# def computeCartesianPath(msg):
    
#     waypoints = []
#     for pose_i in msg.poses:
#         waypoints.append(copy.deepcopy(pose_i.pose))
   

#     # We want the Cartesian path to be interpolated at a resolution of 1 cm
#     # which is why we will specify 0.01 as the eef_step in Cartesian
#     # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
#     (plan, fraction) = move_group.compute_cartesian_path(
#                                        waypoints,   # waypoints to follow
#                                        0.1,        # eef_step
#                                        0.0)         # jump_threshold

#     #displaying plan
#     display_trajectory = moveit_msgs.msg.DisplayTrajectory()
#     display_trajectory.trajectory_start = robot.get_current_pose()
#     display_trajectory.trajectory.append(plan)
#     display_trajectory_publisher.publish(display_trajectory)

#     move_group.execute(plan,wait=True)
# sub = rospy.Subscriber("/door_planner/door_handle_path", Path, computeCartesianPath)

msg = rospy.wait_for_message("/door_planner/door_handle_path", Message, timeout=2000)
waypoints = []
for pose_i in msg.door_handle_pose.poses:
    waypoints.append(copy.deepcopy(pose_i.pose))


# We want the Cartesian path to be interpolated at a resolution of 1 cm
# which is why we will specify 0.01 as the eef_step in Cartesian
# translation.  We will disable the jump threshold by setting it to 0.0 disabling:
(plan, fraction) = move_group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.1,        # eef_step
                                    0.0)         # jump_threshold

#displaying plan
# display_trajectory = moveit_msgs.msg.DisplayTrajectory()
# display_trajectory.trajectory_start = robot.get_current_pose()
# display_trajectory.trajectory.append(plan)
# display_trajectory_publisher.publish(display_trajectory)

move_group.execute(plan,wait=True)
# pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

# speed = Twist()

# r = rospy.Rate(4)




# ## 매니퓰레이터 변수 선언










# #통합제어

# joint_goal = move_group.get_current_joint_values()


# #변수 선언
# mobile_joints = [-pi/3, 0.5]
# joint_goal_list = [pi*90/180, pi*-130/180 , pi*111/180, pi*-68/180, pi*-90/180, 0] #home pose


# #모바일 제어
# goal_x_dist = mobile_joints[1] #meter
# angle_to_goal = mobile_joints[0]
# goal_distance = 0
# r.sleep()

# if theta-angle_to_goal > 0:
#     while abs(theta-angle_to_goal) > 0.01:

#         if abs(theta-angle_to_goal) < 0.2:
#             speed.angular.z = 0.02
#         else:
#             speed.angular.z = 0.2

#         pub.publish(speed)
#         r.sleep()
#         print "theta:",theta,"goal_th:",angle_to_goal
# else:
#     while abs(theta-angle_to_goal) > 0.01:

#         if abs(theta-angle_to_goal) < 0.2:
#             speed.angular.z = -0.02
#         else:
#             speed.angular.z = -0.2

#         pub.publish(speed)
#         r.sleep()
#         print "theta:",theta,"goal_th:",angle_to_goal    


# init_x = x
# init_y = y

# while abs(goal_x_dist - goal_distance) >0.01:

#     current_dist = abs(sqrt((x-init_x) ** 2 + (y-init_y) ** 2))
#     goal_distance = current_dist

#     if abs(goal_x_dist - goal_distance) < 0.1:
#         speed.linear.x = 0.02
#         speed.angular.z = 0
#     else:
#         speed.linear.x = 0.3
#         speed.angular.z = 0
    
#     pub.publish(speed)
#     r.sleep()
#     goal_distance = current_dist
#     print "distance:",goal_distance



# joint_goal_list = [pi*90/180, pi*-130/180 , pi*111/180, pi*-68/180, pi*-90/180, 0] #home pose

# #매니퓰레이터 제어
# joint_goal[0] = joint_goal_list[0]
# joint_goal[1] = joint_goal_list[1]
# joint_goal[2] = joint_goal_list[2]
# joint_goal[3] = joint_goal_list[3]
# joint_goal[4] = joint_goal_list[4]
# joint_goal[5] = joint_goal_list[5]
# move_group.go(joint_goal, wait=True)


# joint_goal_list = [pi/2,0,0,0,0,0] #home pose
# joint_goal[0] = joint_goal_list[0]
# joint_goal[1] = joint_goal_list[1]
# joint_goal[2] = joint_goal_list[2]
# joint_goal[3] = joint_goal_list[3]
# joint_goal[4] = joint_goal_list[4]
# joint_goal[5] = joint_goal_list[5]
# move_group.go(joint_goal, wait=True)

# move_group.stop()
















 