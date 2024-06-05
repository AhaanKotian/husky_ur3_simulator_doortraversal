#include "Door.h"
Door::Door()
{
    /* initialise all member variables
    Got a start: -0.03, -0.07, and a goal: 1.87, -0.03 (hinge) ; Setting goal: Frame:odom, Position(1.896, 0.073, 0.000)
    Got a start: -0.08, -0.10, and a goal: 1.85, 0.68 (handle) ; Setting goal: Frame:odom, Position(1.899, 0.873, 0.000)
    */
    door_hinge_point.header.frame_id = "map";
    door_hinge_point.header.stamp = ros::Time();
    // door_hinge_point.point.x = 0.20;
    // door_hinge_point.point.y = 1.80;
    
    // door_hinge_point.point.x = 1.80;
    // door_hinge_point.point.y = -0.20;
    
    door_hinge_point.point.x = 0.775;
    door_hinge_point.point.y = -0.925;
    door_hinge_point.point.z = 0.0;

    door_handle_point.header.frame_id = "map";
    door_handle_point.header.stamp = ros::Time();
    // door_handle_point.point.x = -0.71;//for max door point (door_handle_point.point.x + 0.08)//door_handle_point.point.x = 1.80;
    // door_handle_point.point.y = 1.80;//door_handle_point.point.y = 0.71;//for max door point (door_handle_point.point.y + 0.08) 
    
    // door_handle_point.point.x = 1.80;
    // door_handle_point.point.y = 0.71;
    
    door_handle_point.point.x = 0.775;
    door_handle_point.point.y = -0.425;
    door_handle_point.point.z = 0.0;

    length = sqrt(pow((door_handle_point.point.x - door_hinge_point.point.x),2) + pow((door_handle_point.point.y - door_hinge_point.point.y),2));//=0.5m
    
    // length = sqrt(pow((door_handle_point.point.x - door_hinge_point.point.x),2) + pow(((door_handle_point.point.y+0.08) - door_hinge_point.point.y),2));//=0.71m

    // length = sqrt(pow(((door_handle_point.point.x-0.08) - door_hinge_point.point.x),2) + pow(((door_handle_point.point.y) - door_hinge_point.point.y),2));//=0.71m
    
    radius = hypot((door_handle_point.point.x - door_hinge_point.point.x),(door_handle_point.point.y - door_hinge_point.point.y));
    max_door_angle_degree = 135.0;
};