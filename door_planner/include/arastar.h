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
#include "state.h"

using namespace std;

struct comp
{
    bool operator()(const State& s1, const State& s2);
};
class ARAStar
{
public:
    //bool compare(const State& s1, const State& s2);
    double goal_wx, goal_wy;
    double epsilon;
    set<State,comp> open;
    set<State,comp> incons;
    set<State,comp> closed;
    State start,goal;
    State* succ;
    map<int,State*> visited;
    map<int,int> pred;
    costmap_2d::Costmap2D* costmap;
    unsigned char* map_;
    unsigned int start_idx_oned,goal_idx_oned;
    unsigned int nx,ny;//number of cells in x direction and y direction in map 
    ARAStar();
    ARAStar(int start_idx_oned, int goal_idx_oned, costmap_2d::Costmap2D* costmap_, double epsilon);
    vector<int> compute_successors_idx_set(State s);
    vector<int> compute_idx_path();
    unsigned int fvalue(State s);
    void improvePath();
    vector<int> search();
    double heuristic(int idx);
    double cost(State s, State succ);
    bool in_closed(State* succ);
};


