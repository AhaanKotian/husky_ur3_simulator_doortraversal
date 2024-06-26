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
x = 0.0
y = 0.0 
theta = 0.0
global_plan_pub = rospy.Publisher("/global_plan", Path, queue_size = 1)

ik_solver = IK("ur3_base_link","ee_link")
seed_state = [0.0] * ik_solver.number_of_joints
goal_state = [0.0] * ik_solver.number_of_joints

group_name = "ur3_manipulator"
move_group = MoveGroupCommander(group_name)
FIXED_FRAME = 'odom'

door_mp_goal = PoseStamped()
door_mp_goal.header.frame_id = "map"
d = 0.2
door_mp_goal.pose.position.x = 0.775
door_mp_goal.pose.position.y = -0.675
door_mp_goal.pose.orientation.w = 1.0

def get_global_plan(start,goal):
    # Create service client
    global response
    client = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
    # Create request message
    print("start(x,y) = ({},{}) & goal(x,y) = ({},{})".format(start.pose.position.x,start.pose.position.y,goal.pose.position.x,goal.pose.position.y))
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
                print("Waypoint pose: ",
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
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def move_base(a,b):
    
    sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    speed = Twist()
    r = rospy.Rate(4)

    goal = Point()
    goal.x = a
    goal.y = b

    arrival_radius = 0.05

    while (goal.x-x)**2 + (goal.y-y)**2 >= arrival_radius**2 :
    #while abs(goal.x-x) >0.1 or abs(goal.y-y) >0.1 or abs(angle_to_goal-theta) >0.1 : #가까의 범위가 0.3이내로 들어오면 break.
        
        inc_x = goal.x -x
        inc_y = goal.y -y
        angle_to_goal = atan2(inc_y,inc_x)

        if abs(angle_to_goal - theta) > 5*pi/180:
            speed.linear.x = 0.0
            speed.angular.z = 0.3
            if abs(angle_to_goal - theta) < 5*pi/180:        # 0.5이내로 들어오면 속도를 매우 줄여서 목표점을 지나쳐버리는 일이 없도록함.
                speed.angular.z = 0.03
                speed.linear.x = 0.0

        
        else:
            speed.linear.x = 0.2
            speed.angular.z = 0.0
            if abs(goal.x-x) <0.3 and abs(goal.y-y)<0.3:    #x,y val이 0.3이내로 들어오면 속도 매우 줄임.
                speed.angular.x = 0.05
                speed.angular.z = 0.0

        #print goal.x-x, goal.y-y, angle_to_goal-theta

        pub.publish(speed)
        #r.sleep() 

    # final_angle_to_goal = 0
    # while abs(final_angle_to_goal - theta) > 0.02:
    #     if abs(final_angle_to_goal - theta) > 0.3:
    #         speed.linear.x = 0
    #         speed.angular.z = 0.3
    #     else:
    #         speed.linear.x = 0
    #         speed.angular.z = 0.1           

    #     pub.publish(speed)
        r.sleep()           

    print ('mobile robot movement complete!')

    return x,y

def manipulation(base_pose_i):

    i=0
    for pose_i in msg.robot_pose.poses:
        if pose_i.pose.position.x == base_pose_i.pose.position.x and pose_i.pose.position.y == base_pose_i.pose.position.y:
            break
        i = i+1
    if len(msg.door_handle_pose.poses) == i:
        return
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
    # i=0
    # for pose_i in response.plan.poses:
    #     move_base(pose_i.pose.position.x,pose_i.pose.position.y)
    #     manipulation(pose_i)
    #     rospy.sleep(5)
    move_base(response.plan.poses[0].pose.position.x,response.plan.poses[0].pose.position.y)
    rospy.sleep(5)
        
        
    # move_base(response.plan.poses[2].pose.position.x,response.plan.poses[2].pose.position.y)

if __name__ == "__main__":

    #GROUP_NAME_GRIPPER = "NAME OF GRIPPER"
    roscpp_initialize(sys.argv)
    rospy.init_node('door_opening_s2', anonymous=True)
    robot = RobotCommander()
    scene = PlanningSceneInterface()
    start = PoseStamped()
    start.header.frame_id = "map"
    start.pose.position.x = -0.075
    start.pose.position.y = -0.025
    start.pose.orientation.w = 1.0
    final_goal = PoseStamped()
    final_goal.header.frame_id = "map"
    final_goal.pose.position.x = 1.525
    final_goal.pose.position.y = -1.425
    final_goal.pose.orientation.w = 1.0

    start_i = PoseStamped()
    start_i.header.frame_id = start.header.frame_id
    start_i.pose.position.x = start.pose.position.x
    start_i.pose.position.y = start.pose.position.y
    start_i.pose.orientation.w = start.pose.orientation.w
    
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.pose.orientation.w = 1.0
    while math.hypot((start_i.pose.position.x - final_goal.pose.position.x),(start_i.pose.position.y - final_goal.pose.position.y)) >= 0.1 :
        print("goal(x,y) = ({},{}) & start_i(x,y) = ({},{})".format(goal.pose.position.x,goal.pose.position.y,start_i.pose.position.x,start_i.pose.position.y))
        if start_i.pose.position.x < door_mp_goal.pose.position.x :
            D = math.hypot((door_mp_goal.pose.position.x - start_i.pose.position.x),(door_mp_goal.pose.position.y - start_i.pose.position.y))
            goal.pose.position.x = start_i.pose.position.x + (d * (door_mp_goal.pose.position.x - start_i.pose.position.x))/D
            goal.pose.position.y = start_i.pose.position.y + (d * (door_mp_goal.pose.position.y - start_i.pose.position.y))/D
        else :
            D = math.hypot((final_goal.pose.position.x - start_i.pose.position.x),(final_goal.pose.position.y - start_i.pose.position.y))
            goal.pose.position.x = start_i.pose.position.x + (d * (final_goal.pose.position.x - start_i.pose.position.x))/D
            goal.pose.position.y = start_i.pose.position.y + (d * (final_goal.pose.position.y - start_i.pose.position.y))/D
        print("goal'(x,y) = ({},{}) & start_i(x,y) = ({},{})".format(goal.pose.position.x,goal.pose.position.y,start_i.pose.position.x,start_i.pose.position.y))
        t1 = threading.Thread(target=get_global_plan, args=[start_i,goal], name='t1')
        t2 = threading.Thread(target=get_door_handle_path, name='t2')
        
        t1.start()
        t2.start()

        t1.join()
        t2.join()

        move_to_base_waypoints()

        start_i = copy.deepcopy(response.plan.poses[0])

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                DisplayTrajectory,
                                                queue_size=20)
    move_group.go([1.57,-2.27,1.93,-1.19,-1.57,0], wait=True) 

    




 