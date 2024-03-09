#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
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

using namespace std;

class State
{
public:
    unsigned int idx;//index of node in 1D array map[]
    double g;// path length from start
    double h;//heuristic i.e. diagonal distance to goal
    // bool visited = false;
    // costmap_2d::Costmap2D* costmap_;
    double epsilon;
    State();
    State(int idx, double g, double h, double epsilon);
};
