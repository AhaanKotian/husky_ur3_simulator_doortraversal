#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
import copy, math
import tf
import threading
import tf2_ros
from math import pi, radians, degrees, atan2, sqrt
from moveit_commander import MoveGroupCommander, RobotCommander 
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Twist
from tf2_geometry_msgs import PointStamped
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, MoveItErrorCodes, DisplayTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import random
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import String
from nav_msgs.msg import Path, Odometry
from nav_msgs.srv import GetPlan
from robot_to_door_handle_msg.msg import Message
from trac_ik_python.trac_ik import IK

response = door_handle_waypoints = msg = 0
curr_x = goal_x = 0.0
curr_y = goal_y = 0.0 
curr_theta = goal_theta = 0.0
global_plan_pub = rospy.Publisher("/global_plan", Path, queue_size = 1)

ik_solver = IK("ur3_base_link","ee_link")
seed_state = [0.0] * ik_solver.number_of_joints
goal_state = [0.0] * ik_solver.number_of_joints


group_name = "ur3_manipulator"
move_group = MoveGroupCommander(group_name)
FIXED_FRAME = 'map'

# PID coefficients
kp_linear = 1.0
ki_linear = 0.0
kd_linear = 0.1

kp_angular = 1.0
ki_angular = 0.0
kd_angular = 0.1

# PID variables
prev_error_linear = 0.0
integral_linear = 0.0

prev_error_angular = 0.0
integral_angular = 0.0

max_linear_speed = 0.5  # m/s
max_angular_speed = 0.5  # rad/s

def get_global_plan(start,goal):
    # Create service client
    global response
    client = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
    # Create request message
    
    tolerance = 0.1

    # Call service
    try:
        response = client(start, goal, tolerance)
        # Print response message
        print("Got plan with %d waypoints.", len(response.plan.poses))
        print("response.plan.header.frame_id = ",response.plan.header.frame_id)
        # Print the pose and orientation of the first waypoint
        if response.plan.poses:
            for pose_i in response.plan.poses:
                print("Waypoint pose: %f, %f, %f, %f, %f, %f, %f",
                                pose_i.pose.position.x,
                                pose_i.pose.position.y,
                                pose_i.pose.position.z,
                                pose_i.pose.orientation.w,
                                pose_i.pose.orientation.x,
                                pose_i.pose.orientation.y,
                                pose_i.pose.orientation.z)
            global global_plan_pub
            global_plan_pub.publish(response.plan.header,response.plan.poses)
    except rospy.ServiceException as e:
        print("Failed to call service /move_base/make_plan: %s", e)


def get_door_handle_path():
    global msg
    msg = rospy.wait_for_message("/door_planner/door_handle_path", Message, timeout=5000)
    

def newOdom(msg):
    global curr_x, curr_y, curr_theta, goal_x, goal_y, goal_theta

    curr_x = msg.pose.pose.position.x
    curr_y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (curr_roll, curr_pitch, curr_theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    r = rospy.Rate(4)

    
    # Calculate the distance and angle to the goal
    distance = math.sqrt((goal_x - curr_x) ** 2 + (goal_y - curr_y) ** 2)
    angle_to_goal = math.atan2(goal_y - curr_y, goal_x - curr_x)

    # Calculate the errors
    error_linear = distance
    error_angular = angle_to_goal - curr_theta

    # Normalize the angular error to the range [-pi, pi]
    error_angular = (error_angular + math.pi) % (2 * math.pi) - math.pi

    global integral_linear, derivative_linear, linear_output, prev_error_linear
    global integral_angular, derivative_angular, angular_output, prev_error_angular

    # PID control for linear velocity
    integral_linear += error_linear
    derivative_linear = error_linear - prev_error_linear
    linear_output = (kp_linear * error_linear +
                        ki_linear * integral_linear +
                        kd_linear * derivative_linear)
    prev_error_linear = error_linear
        

    # PID control for angular velocity
    integral_angular += error_angular
    derivative_angular = error_angular - prev_error_angular
    angular_output = (kp_angular * error_angular +
                        ki_angular * integral_angular +
                        kd_angular * derivative_angular)
    prev_error_angular = error_angular
    
    # Limit the velocities
    linear_output = max(min(linear_output, max_linear_speed), -max_linear_speed)
    angular_output = max(min(angular_output, max_angular_speed), -max_angular_speed)

    # Create and publish the Twist message
    twist = Twist()
    twist.linear.x = linear_output
    twist.angular.z = angular_output
    pub.publish(twist)
    
    # Stop if within a threshold of the goal
    if distance < 0.1 and abs(error_angular) < 0.1:
        print("Local Goal reached")
        r.sleep()

def move_base(x,y,theta):
    global goal_x,goal_y,goal_theta
    goal_x = x
    goal_y = y 
    goal_theta = theta 
    sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)  
    print ('mobile robot movement complete!')


def manipulation(base_pose_i):
    
    i=0
    for pose_i in msg.robot_pose.poses:
        if pose_i.pose.position.x == base_pose_i.pose.position.x and pose_i.pose.position.y == base_pose_i.pose.position.y:
            break
        i = i+1
    if msg.door_handle_pose.poses[i].pose.position.y < -1.28:
        return
    
    # listener = tf.TransformListener()
    # listener.waitForTransform("/map","ur3_base_link",rospy.Time.now(),rospy.Duration(10.0))
    # door_handle_pose_transformed = listener.transformPose("ur3_base_link",msg.door_handle_pose.poses[i])

    door_handle_ps = PointStamped()
    # door_handle_ps.header.stamp = msg.door_handle_pose.poses[i].header.stamp
    door_handle_ps.header.stamp = rospy.Time(0)
    door_handle_ps.header.frame_id = msg.door_handle_pose.poses[i].header.frame_id
    door_handle_ps.point.x = msg.door_handle_pose.poses[i].pose.position.x
    door_handle_ps.point.y = msg.door_handle_pose.poses[i].pose.position.y
    door_handle_ps.point.z = msg.door_handle_pose.poses[i].pose.position.z
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    door_handle_ps_transformed = tfBuffer.transform(door_handle_ps,"ur3_base_link",timeout=rospy.Duration(10.0))
    
    # global goal_state, seed_state
    # goal_state = ik_solver.get_ik(seed_state,
    #             door_handle_pose_transformed.pose.position.x, door_handle_pose_transformed.pose.position.y, door_handle_pose_transformed.pose.position.z,
    #             door_handle_pose_transformed.pose.orientation.x, door_handle_pose_transformed.pose.orientation.y, door_handle_pose_transformed.pose.orientation.z, door_handle_pose_transformed.pose.orientation.w)
    
    global goal_state, seed_state
    goal_state = ik_solver.get_ik(seed_state,
                door_handle_ps_transformed.point.x, door_handle_ps_transformed.point.y, door_handle_ps_transformed.point.z,
                0.0, 0.0, 0.0, 1,0)
    
    print(" base_pose_i(x,y) = ({},{}) & door_handle_pose_transformed(x,y,z) = ({},{},{})".format(base_pose_i.pose.position.x,base_pose_i.pose.position.y, door_handle_ps_transformed.point.x, door_handle_ps_transformed.point.y, door_handle_ps_transformed.point.z))
    print("goal_state",goal_state)
    move_group.go(goal_state,wait=True)
        

def move_to_base_waypoints():
    global response
    i=0
    # for pose_i in response.plan.poses:
    #     move_base(pose_i.pose.position.x,pose_i.pose.position.y)
    #     manipulation(pose_i)
    #     rospy.sleep(5)
    move_base(response.plan.poses[0].pose.position.x,response.plan.poses[0].pose.position.y,response.plan.poses[0].pose.orientation.z)
    rospy.sleep(5)
    # move_base(response.plan.poses[2].pose.position.x,response.plan.poses[2].pose.position.y)

if __name__ == "__main__":

    #GROUP_NAME_GRIPPER = "NAME OF GRIPPER"
    roscpp_initialize(sys.argv)
    rospy.init_node('door_opening_s2', anonymous=True)
    robot = RobotCommander()
    scene = PlanningSceneInterface()
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                DisplayTrajectory,
                                                queue_size=20)
    move_group.go([1.57,-2.27,1.93,-1.19,-1.57,0], wait=True) 

    c = input("Enter 1 for offline planning; 2 for online planning : ")

    start = PoseStamped()
    start.header.frame_id = "map"
    start.pose.position.x = -0.075
    start.pose.position.y = -0.025
    start.pose.orientation.w = 1.0
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.pose.position.x = 1.525
    goal.pose.position.y = -1.425
    goal.pose.orientation.w = 1.0

    start_i = PoseStamped()
    start_i.header.frame_id = start.header.frame_id
    start_i.pose.position.x = start.pose.position.x
    start_i.pose.position.y = start.pose.position.y
    start_i.pose.orientation.w = start.pose.orientation.w

    while math.hypot((start_i.pose.position.x - goal.pose.position.x),(start_i.pose.position.y - goal.pose.position.y)) >= 0.1 :
        print("goal(x,y) = ({},{}) & start_i(x,y) = ({},{})".format(goal.pose.position.x,goal.pose.position.y,start_i.pose.position.x,start_i.pose.position.y))
        
        t1 = threading.Thread(target=get_global_plan, args=[start_i,goal], name='t1')
        t2 = threading.Thread(target=get_door_handle_path, name='t2')
        
        t1.start()
        t2.start()

        t1.join()
        t2.join()

        move_to_base_waypoints()
        # print("Sleeping")
        # rospy.sleep(100)
        if c==1:
            break
        
        start_i = copy.deepcopy(response.plan.poses[0])

    print(" Done! Goal Reached! ")




 # #! /usr/bin/env python3
# # -*- coding: utf-8 -*-

# import sys
# import rospy
# import copy, math
# import tf
# import tf2_ros
# import threading
# from math import pi, radians, degrees, atan2, sqrt
# from moveit_commander import MoveGroupCommander, RobotCommander 
# from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
# from moveit_commander.conversions import pose_to_list
# # from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Twist
# from tf2_geometry_msgs import PointStamped
# from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, MoveItErrorCodes, DisplayTrajectory
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from tf.transformations import euler_from_quaternion, quaternion_from_euler
# import random
# import actionlib
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# # from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
# from tf.transformations import quaternion_from_euler, euler_from_quaternion
# from std_msgs.msg import String
# from nav_msgs.msg import Path, Odometry
# from nav_msgs.srv import GetPlan
# from robot_to_door_handle_msg.msg import Message
# from trac_ik_python.trac_ik import IK
# roscpp_initialize(sys.argv)
# rospy.init_node('test_door_opening_s2', anonymous=True)

# # test = PointStamped()
# # test.header.stamp = rospy.Time(0)
# # test.header.frame_id = "map"
# # test.point.x = 0.563691
# # test.point.y = -0.471846
# # test.point.z = 0.5
# # tfBuffer = tf2_ros.Buffer()
# # listener = tf2_ros.TransformListener(tfBuffer)
# # test_transformed = tfBuffer.transform(test,"ur3_base_link",timeout=rospy.Duration(10.0))
# # print(" test_transformed(x,y,z) = ({},{},{})".format(test_transformed.point.x, test_transformed.point.y, test_transformed.point.z))

# test = PointStamped()
# test.header.stamp = rospy.Time(0)
# test.header.frame_id = "ur3_base_link"
# test.point.x = float(input("x = "))
# test.point.y = float(input("y = "))
# test.point.z = float(input("z = "))
# # tfBuffer = tf2_ros.Buffer()
# # listener = tf2_ros.TransformListener(tfBuffer)
# # test_transformed = tfBuffer.transform(test,"ur3_base_link",timeout=rospy.Duration(10.0))
# # print(" test_transformed(x,y,z) = ({},{},{})".format(test_transformed.point.x, test_transformed.point.y, test_transformed.point.z))

# ik_solver = IK("ur3_base_link","ee_link")
# seed_state = [0.0] * ik_solver.number_of_joints
# goal_state = [0.0] * ik_solver.number_of_joints


# group_name = "ur3_manipulator"
# move_group = MoveGroupCommander(group_name)
# FIXED_FRAME = 'map'

# goal_state, seed_state
# goal_state = ik_solver.get_ik(seed_state,
#             test.point.x, test.point.y, test.point.z,
#             0.0, 0.0, 0.0, 1,0)

# # print(" base_pose_i(x,y) = ({},{}) & door_handle_pose_transformed(x,y,z) = ({},{},{})".format(base_pose_i.pose.position.x,base_pose_i.pose.position.y, door_handle_ps_transformed.point.x, door_handle_ps_transformed.point.y, door_handle_ps_transformed.point.z))
# print("goal_state",goal_state)
# move_group.go(goal_state,wait=True)

# # test = PoseStamped()
# # test.header.stamp = rospy.Time(0)
# # test.header.frame_id = "map"
# # test.pose.position.x = 0.563691
# # test.pose.position.y = -0.471846
# # test.pose.position.z = 0.5
# # test.pose.orientation.x = 0.0
# # test.pose.orientation.y = 0.0
# # test.pose.orientation.z = 0.0
# # test.pose.orientation.w = 1.0
# # listener = tf.TransformListener()
# # listener.waitForTransform("/map","ur3_base_link",rospy.Time.now(),rospy.Duration(10.0))
# # door_handle_pose_transformed = listener.transformPose("ur3_base_link",test)
# # print(" test(x,y,z,r,p,y) = ({},{},{},{},{},{})".format(test.pose.position.x, test.pose.position.y, test.pose.position.z, test.pose.orientation.x, test.pose.orientation.y, test.pose.orientation.z))
