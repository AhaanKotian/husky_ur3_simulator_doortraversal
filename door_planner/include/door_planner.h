/** include the libraries you need in your planner here */
/** for global path planner interface */
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
#include <map>
#include <algorithm>
#include <cmath>
#include <random>
#include "arastar.h"

using std::string;

#ifndef GLOBAL_PLANNER_CPP
#define GLOBAL_PLANNER_CPP

namespace door_global_planner 
{
    class DoorPlanner : public nav_core::BaseGlobalPlanner 
    {
    public:

        DoorPlanner();
        DoorPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        /** overridden classes from interface nav_core::BaseGlobalPlanner **/
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped& start,
                        const geometry_msgs::PoseStamped& goal,
                        std::vector<geometry_msgs::PoseStamped>& plan
                    );

    private:

        costmap_2d::Costmap2DROS* costmap_ros_;
        double step_size_, min_dist_from_robot_;
        costmap_2d::Costmap2D* costmap_;
        unsigned char* map;
        base_local_planner::WorldModel* world_model_; 
        std::default_random_engine generator;
        unsigned int map_length;
        bool initialized_;
    };

};
#endif