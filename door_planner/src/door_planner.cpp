#include <pluginlib/class_list_macros.h>
#include "door_planner.h"
PLUGINLIB_EXPORT_CLASS(door_global_planner::DoorPlanner, nav_core::BaseGlobalPlanner)
namespace door_global_planner
{   
    DoorPlanner::DoorPlanner() : costmap_ros_(NULL), initialized_(false)
    {

    }

    DoorPlanner::DoorPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) : costmap_ros_(NULL), initialized_(false)
    {
        initialize(name, costmap_ros);
    }

    void DoorPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
        if(!initialized_)
        {
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();
            ros::NodeHandle private_nh("~/" + name);
            private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);

            initialized_ = true;
        }
        else
        ROS_WARN("This planner has already been initialized... doing nothing");
    }


    bool DoorPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
    {
        
        if(!initialized_)
        {
            ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
            return false;
        }

        ROS_INFO("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

        plan.clear();
        costmap_ = costmap_ros_->getCostmap();
        map = costmap_->getCharMap();
        unsigned int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();
        //ARAStar::nx = costmap_->getSizeInCellsX(), ARAStar::ny = costmap_->getSizeInCellsY();
        map_length = nx*ny;
        
        double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
        
        unsigned int mx,my;

        if(goal.header.frame_id != costmap_ros_->getGlobalFrameID())
        { 
            double start_wx = start.pose.position.x;
            double start_wy = start.pose.position.y;
            unsigned int start_idx_oned;
            if(costmap_->worldToMap(start_wx,start_wy,mx,my))
                start_idx_oned = costmap_->getIndex(mx,my);
            
            ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.", 
                costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
            return false;
        }

        double goal_wx = goal.pose.position.x;
        double goal_wy = goal.pose.position.y;
        unsigned int goal_idx_oned;
        if(costmap_->worldToMap(goal_wx,goal_wy,mx,my))
            goal_idx_oned = costmap_->getIndex(mx,my);

        double start_wx = start.pose.position.x;
        double start_wy = start.pose.position.y;
        unsigned int start_idx_oned;
        if(costmap_->worldToMap(start_wx,start_wy,mx,my))
            start_idx_oned = costmap_->getIndex(mx,my);
        
        ROS_INFO_STREAM(" start idx = "<<start_idx_oned);
        ROS_INFO_STREAM(" goal idx = "<<goal_idx_oned);

        //std::vector<Node> path;
         
        int destination = goal_idx_oned;
        //goal_node.idx = destination;
        int source = start_idx_oned;

        double epsilon = 5.0;
        ARAStar planner(source,destination,costmap_,epsilon);
        vector<int> path;
        path = planner.search();

        ROS_INFO_STREAM(" PATH.SIZE() = "<<path.size());
        ros::Time plan_time = ros::Time::now();
        for(int i=0;i<path.size();i++)
        {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = plan_time;
            pose.header.frame_id = start.header.frame_id;
            unsigned int mx1,my1;
            costmap_->indexToCells(path[i],mx1,my1);
            double wx,wy;
            costmap_->mapToWorld(mx1,my1,wx,wy);
            pose.pose.position.x = wx;
            pose.pose.position.y = wy;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            plan.push_back(pose);
            ROS_INFO_STREAM(" x = "<<plan[i].pose.position.x<<" AND y = "<<plan[i].pose.position.y<<" for path["<<i<<"] = "<<path[i]);
        }

        return true;
    }
};