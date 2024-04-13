#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <iostream>
#include <vector>
#include <queue>
#include <set>
#include <map>
#include <algorithm>
#include <cmath>
#include <random>

class Door
{
public:
    geometry_msgs::PointStamped door_hinge_point;
    geometry_msgs::PointStamped door_handle_point;
    double length,radius;
    double max_door_angle_degree;// data type is in because 1 degree discretization of door angle
    Door();
};