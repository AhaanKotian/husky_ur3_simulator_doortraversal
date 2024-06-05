#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include "/home/ahaan/iitb_ws/devel/include/robot_to_door_handle_msg/Message.h"
#include <geometry_msgs/Point.h>
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
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>
#include "state.h"
#include "Door.h"

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
    // map<unsigned int,State*> visited;
    // map<unsigned int,unsigned int> pred;
    // map<unsigned int,unsigned int> idx_to_door_angle;
    // vector<unsigned int> door_angle_path;
    map< pair<unsigned int,unsigned int> , State* > visited;
    map< pair<unsigned int,unsigned int> , pair<unsigned int,unsigned int> > pred;
    map< pair<unsigned int,unsigned int> , unsigned int > idx_to_door_angle;
    vector<unsigned int> door_angle_path;
    std::unique_ptr<ros::NodeHandle> rosNode;
    ros::Publisher robot_door_handle_path_pub;
    robot_to_door_handle_msg::Message msg;
    costmap_2d::Costmap2D* costmap;
    unsigned char* map_;
    unsigned int start_idx_oned,goal_idx_oned;
    unsigned int nx,ny;//number of cells in x direction and y direction in map 
    Door door;
    tf::TransformListener listener;
    tf::StampedTransform transform;double transform_x,transform_y;
    double reach;

    //for visualisation
    visualization_msgs::Marker tree;
    visualization_msgs::Marker poly;
    ros::Publisher tree_pub;
    ros::Publisher poly_pub;
    
    ARAStar();
    ARAStar(int start_idx_oned, int goal_idx_oned, costmap_2d::Costmap2D* costmap_, double epsilon);
    
    vector<unsigned int> compute_successors_idx_set(State s);
    vector<unsigned int> compute_idx_path();
    double fvalue(State s);
    void improvePath();
    vector<geometry_msgs::PoseStamped> search();
    double heuristic(unsigned int idx,double theta,unsigned int a);
    double cost(State s, State succ);
    bool is_occupied(unsigned int succ_idx, unsigned int s_idx);
    bool in_closed(State* succ);
    bool is_action_feasible(State s,unsigned int succ_idx,vector<unsigned int> succ_rda,unsigned int succ_a);

    vector<pair<unsigned int , vector<unsigned int> > > compute_a(unsigned int idx,double theta);
    double compute_manipulator_dist(unsigned int idx, int c);
    bool in_first_room(unsigned int idx);
    bool inside_line_of_door(unsigned int idx, unsigned int door_angle);

    vector<pair<unsigned int , vector<unsigned int> > > compute_reachable_door_angles(unsigned int idx, double theta);
    bool is_door_handle_reachable( unsigned int idx, double curr_ur3_wx, double curr_ur3_wy, unsigned int door_angle);
    bool door_collision_with_robot(vector<pair<double,double> > footprint, double angle);
    bool check_point_in_footprint(double x, double y, vector<pair<double,double> > footprint);
    vector<pair<double,double> > compute_robot_footprint(double curr_wx, double curr_wy, double theta);
    void compute_cartesian_door_handle_path();
    
    double compute_heading_angle(unsigned int s1_idx, unsigned int s2_idx);

    //FOR VISUALISATION AND DEBUGGING
    void init_line(visualization_msgs::Marker* line_msg);
    void pub_line(visualization_msgs::Marker* line_msg, ros::Publisher* line_pub, double x1, double y1, double x2, double y2,bool b);
};


