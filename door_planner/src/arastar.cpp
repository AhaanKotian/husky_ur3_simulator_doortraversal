#include "arastar.h"
using namespace std;
ARAStar::ARAStar()
{

};
ARAStar::ARAStar(int start_idx_oned, int goal_idx_oned, costmap_2d::Costmap2D* costmap_, double epsilon) : start_idx_oned(start_idx_oned), goal_idx_oned(goal_idx_oned), costmap(costmap_), epsilon(epsilon)
{
    nx = costmap->getSizeInCellsX(); ny = costmap->getSizeInCellsY();
    //initialise start state
    start.idx = start_idx_oned;
    start.g = 0.0;
    start.h = heuristic(start_idx_oned,0.0,0);
    start.epsilon = epsilon;
    start.a = 0;
    //initialise goal state
    goal.idx = goal_idx_oned;
    goal.g = double(INT16_MAX);
    goal.h = 0;
    goal.epsilon = epsilon;
    goal.a = 4; 
    unsigned int mx,my;
    costmap->indexToCells(goal_idx_oned,mx,my);
    costmap->mapToWorld(mx,my,goal_wx,goal_wy);
    // visited.resize(nx*ny,NULL);
    map_ = costmap->getCharMap();
    ROS_INFO_STREAM("MAP SIZE = "<<nx*ny);
    reach = 1;//0.5m = 500mm = 50cm; from ur3_base_link

    //base_link ---> ur3_base_link transform
    try
    {
        listener.waitForTransform("/ur3_base_link","/base_link",ros::Time(0),ros::Duration(10.0));
        listener.lookupTransform("/ur3_base_link","/base_link",ros::Time(0),transform);
    }
    catch(tf::TransformException ex)
    {
        ROS_INFO_STREAM(" Tranform error : "<<ex.what());
        // transform_x = 0.33;
        // transform_y = 0.0;

    }
    ROS_INFO_STREAM("transform.getOrigin().x() = "<<transform.getOrigin().x()<<" & transform.getOrigin().y() = "<<transform.getOrigin().y());
    rosNode.reset(new ros::NodeHandle("door_planner"));
    robot_door_handle_path_pub = rosNode->advertise<robot_to_door_handle_msg::Message>("door_handle_path",1);

    //Testing
    ROS_INFO_STREAM("in_first_room(start.idx) = "<<in_first_room(start.idx)<<" & in_first_room(goal.idx) = "<<in_first_room(goal.idx)<<" & in_first_room(start.idx+1) = "<<in_first_room(start.idx+1)<<" & in_first_room(goal.idx+1) = "<<in_first_room(goal.idx+1));
    ROS_INFO_STREAM("inside_line_of_door(start.idx) = "<<inside_line_of_door(start.idx,0.0)<<" & inside_line_of_door(goal.idx) = "<<inside_line_of_door(goal.idx,0.0));
    // ROS_INFO_STREAM("compute_a(goal.idx) = "<<compute_a(goal.idx,0.0));
    ROS_INFO_STREAM("start.h = "<<start.h);

    // unsigned int idx_check;
    // costmap->worldToMap(double(1.87),double(0.60),mx,my);
    // idx_check = costmap->getIndex(mx,my);
    ROS_INFO_STREAM("GOAL TEST :");
    vector<pair<unsigned int , vector<unsigned int> > > test;
    test = compute_a(goal_idx_oned,0.0);
    for(pair<unsigned int , vector<unsigned int> > i:test)
    {
        ROS_INFO_STREAM("a = "<<i.first<<" & rda_range = [");
        for(unsigned int i_rda:i.second)
            ROS_INFO_STREAM(i_rda);
    }
    ROS_INFO_STREAM("]");

    //compute_robot_footprint() test
    vector<pair<double,double> > test_footprint = compute_robot_footprint(1.87,0.60,M_PI_2);
    
    // is_occupied() test
    is_occupied(goal_idx_oned,start_idx_oned);
    
    
};
double ARAStar::heuristic(unsigned int idx, double theta, unsigned int a)
{
    unsigned int mx,my;
    costmap->indexToCells(idx,mx,my);
    double curr_wx,curr_wy;
    costmap->mapToWorld(mx,my,curr_wx,curr_wy);
    unsigned int ai=a;
    double h,d;
    double x_ifodh , y_ifodh;// x_ifodh,y_ifodh = x coordinate in map frame, y coordinate in map frame (ifodh : in front of door handle)
    x_ifodh = door.door_handle_point.point.x - 0.5;// 0.5 comes from robot_footprint in husky_ur3_navigation/config/costmap_common_params.yaml
    y_ifodh = door.door_handle_point.point.y;
    double x_osod = x_ifodh, y_osod = y_ifodh;// osod: outside sweep of door
    while( sqrt(pow((door.door_hinge_point.point.x - x_osod),2) + pow((door.door_hinge_point.point.y - y_osod),2)) < door.length )
        x_osod-=0.05;
    x_osod-= 0.5;
    // firstly find mid point of line joined by door_handle_point and door_hinge_point
    double x_midpt, y_midpt;
    x_midpt = (door.door_handle_point.point.x + door.door_hinge_point.point.x)/2;
    y_midpt = (door.door_handle_point.point.y + door.door_hinge_point.point.y)/2;
    // now find desired point i.e. coordinates of base-link that has crossed over the doorsill.
    double x_cotd , y_cotd; // cotd : crossed over the doorsill
    x_cotd = x_midpt + 0.5;
    y_cotd = y_midpt;
    while(ai<=4)
    {
        if(ai==0)
        {
            d = sqrt(pow((x_ifodh-curr_wx),2) + pow((y_ifodh-curr_wy),2));
            //ROS_INFO_STREAM("In ai==0");
        }
        else if(ai==1)
        {
            if(a==1)
                {d = sqrt(pow((x_osod-curr_wx),2) + pow((y_osod-curr_wy),2));}
            else
                {d = sqrt(pow((x_osod-x_ifodh),2) + pow((y_osod-y_ifodh),2));}
        }
        else if(ai==2)
        {
            if(a==2)
                {d = sqrt(pow((door.door_handle_point.point.x-curr_wx),2) + pow((door.door_handle_point.point.y-curr_wy),2));}
            else
                {d = sqrt(pow((door.door_handle_point.point.x-x_osod),2) + pow((door.door_handle_point.point.y-y_osod),2));}
        }
        else if(ai==3)
        {
            if(a==3)
                {d = sqrt(pow((x_cotd-curr_wx),2) + pow((y_cotd-curr_wy),2));}
            else
                {d = sqrt(pow((x_cotd-x_midpt),2) + pow((y_cotd-y_midpt),2));}
        }
        else if(ai==4)
        {
            if(a==4)
                {d = sqrt(pow((goal_wx-curr_wx),2) + pow((goal_wy-curr_wy),2));}
            else
                {d = sqrt(pow((goal_wx-x_cotd),2) + pow((goal_wy-y_cotd),2));}
        }
        h += d;
        ai++;
    }
    // double world_dist = sqrt(pow((goal_wx-curr_wx),2) + pow((goal_wy-curr_wy),2)); //euclidian distance from curr(wx,wy) to goal(wx,wy)
    // return costmap->cellDistance(world_dist); //euclidian distance in terms of cell distance
    //ROS_INFO_STREAM("h = "<<h);
    return h;
};
bool ARAStar::check_point_in_footprint(double x, double y, vector<pair<double,double> > footprint)
{
    int num_vertices = footprint.size();
    bool inside = false;
    pair<double,double> p1 = footprint[0], p2;
 
    // Loop through each edge in the polygon
    for (int i = 1; i <= num_vertices; i++) {
        // Get the next point in the polygon
        p2 = footprint[i % num_vertices];
 
        // Check if the point is above the minimum y
        // coordinate of the edge
        if (y > min(p1.second, p2.second)) {
            // Check if the point is below the maximum y
            // coordinate of the edge
            if (y <= max(p1.second, p2.second)) {
                // Check if the point is to the left of the
                // maximum x coordinate of the edge
                if (x <= max(p1.first, p2.first)) {
                    // Calculate the x-intersection of the
                    // line connecting the point to the edge
                    double x_intersection
                        = (y - p1.second) * (p2.first - p1.first)
                              / (p2.second - p1.second)
                          + p1.first;
 
                    // Check if the point is on the same
                    // line as the edge or to the left of
                    // the x-intersection
                    if (p1.first == p2.first
                        || x <= x_intersection) {
                        // Flip the inside flag
                        inside = !inside;
                        //ROS_INFO_STREAM("inside = "<<inside);
                    }
                }
            }
        }
 
        // Store the current point as the first point for
        // the next iteration
        p1 = p2;
    }
    // ROS_INFO_STREAM("("<<x<<","<<y<<") in [");
    // for(auto f:footprint)
    //     ROS_INFO_STREAM("("<<f.first<<","<<f.second<<")");
    // ROS_INFO_STREAM("] ??? = "<<inside);
    // Return the value of the inside flag
    return inside;
};
bool ARAStar::door_collision_with_robot(vector<pair<double,double> > footprint, double angle)
{
    // ROS_INFO_STREAM("In door_collision_with_robot()");
    double step_size = 0.05,x,y; unsigned int k = 1;
    while((k*step_size)<=door.length)
    {
        x = door.door_hinge_point.point.x + k * step_size * cos((90+angle)*M_PI/180);
        y = door.door_hinge_point.point.y + k * step_size * sin((90+angle)*M_PI/180);
        // ROS_INFO_STREAM("x = "<<door.door_hinge_point.point.x<<" + "<<k<<" * "<<step_size<<" * "<<cos((90+angle)*M_PI/180));
        // ROS_INFO_STREAM("y = "<<door.door_hinge_point.point.y<<" + "<<k<<" * "<<step_size<<" * "<<sin((90+angle)*M_PI/180));
        // ROS_INFO_STREAM("x = "<<x<<" & y = "<<y<<" for angle = "<<angle);
        if(check_point_in_footprint(x,y,footprint))
            return true;
        k++;
    }
    x = door.door_hinge_point.point.x + door.length * cos((90+angle)*M_PI/180);
    y = door.door_hinge_point.point.y + door.length * sin((90+angle)*M_PI/180);
    if(check_point_in_footprint(x,y,footprint))
        return true;

    return false;
};
vector<pair<double,double> > ARAStar::compute_robot_footprint(double curr_wx, double curr_wy, double theta)
{
    vector<pair<double,double> > footprint;//0th index is top_left corner, 1st index is top_right, 2nd index is bottom left, 3rd is bottom right
    pair<double,double> footprint_coordinate;
    double x,y;
    for(int i=0;i<4;i++)
    {
        if(i==0)
        {
            x = 0.5;//considering centre of robot at origin / corodinates of footprint in base_link frame    
            y = 0.35;
        }
        else if(i==1)
        {
            x = 0.5;
            y = -0.35;
        }
        else if(i==2)
        {
            x = -0.5;
            y = 0.35;
        }
        else if(i==3)
        {
            x = -0.5;
            y = -0.35;
        }
        //applying rotation matrix & adding curr_wx & curr_wy to translate it to map_frame from base_link frame
        //ROS_INFO_STREAM("x = "<<x<<"*"<<cos(theta)" - "<<y<<"*"<<sin(theta)<<" + "<<curr_wx);
        footprint_coordinate.first = x*cos(theta) - y*sin(theta) + curr_wx;
        //ROS_INFO_STREAM("y = "<<x<<"*"<<sin(theta)" - "<<y<<"*"<<cos(theta)<<" + "<<curr_wy);
        footprint_coordinate.second = x*sin(theta) + y*cos(theta) + curr_wy;
        footprint.push_back(footprint_coordinate);
    }
    // ROS_INFO_STREAM("("<<curr_wx<<","<<curr_wy<<");s footprint = [");
    // for(auto f:footprint)
    //     ROS_INFO_STREAM("("<<f.first<<","<<f.second<<")");
    // ROS_INFO_STREAM("]");
    return footprint;
};
// void ARAStar::compute_reachable_door_angles(int idx)
vector<pair<unsigned int , vector<unsigned int> > > ARAStar::compute_reachable_door_angles(unsigned int idx, double theta)
{
    // pair<unsigned int,unsigned int> reachable_door_angles_range;
    // pair<unsigned int , pair<unsigned int,unsigned int> > reachable_door_angles_a;
    // vector<pair<unsigned int , pair<unsigned int,unsigned int> > > reachable_door_angles;
    vector<unsigned int> reachable_door_angles_i;
    pair<unsigned int , vector<unsigned int> > reachable_door_angles_a;
    vector<pair<unsigned int , vector<unsigned int> > > reachable_door_angles;
    unsigned int mx,my;
    costmap->indexToCells(idx,mx,my);
    double curr_wx,curr_wy;
    costmap->mapToWorld(mx,my,curr_wx,curr_wy);

    //transformed from base_link's coordinates to ur3_base_link's coordinates
    double curr_ur3_wx,curr_ur3_wy;
    // if(transform.getOrigin().x()>0.32 && transform.getOrigin().x()<0.34 && transform.getOrigin().y()==0.0)
    // {
    //     curr_ur3_wx = curr_wx + transform.getOrigin().x();
    //     curr_ur3_wy = curr_wy + transform.getOrigin().y();
    // }
    // else
    // {
    //     curr_ur3_wx = curr_wx + transform_x;
    //     curr_ur3_wy = curr_wy + transform_y;
    // }
    curr_ur3_wx = curr_wx + transform.getOrigin().x();
    curr_ur3_wy = curr_wy + transform.getOrigin().y();

    double x1, y1, x2, y2;
    double d = hypot( (curr_ur3_wx - door.door_hinge_point.point.x) , (curr_ur3_wy - door.door_hinge_point.point.y) );
    if(d > reach + door.radius)
    {
        if(in_first_room(idx))
            reachable_door_angles_a.first = 0;
        else
            reachable_door_angles_a.first = 4;
        reachable_door_angles_a.second = reachable_door_angles_i;
        reachable_door_angles.push_back(reachable_door_angles_a);
        return reachable_door_angles;
    }
    if(d < abs(reach - door.radius)) // smaller circle completely inside larger circle 
    {
        if(in_first_room(idx))
            reachable_door_angles_a.first = 2;
        else
            reachable_door_angles_a.first = 3;
        reachable_door_angles_a.second = reachable_door_angles_i;
        reachable_door_angles.push_back(reachable_door_angles_a);
        return reachable_door_angles;
    }
    double l = ( pow(door.radius,2) - pow(reach,2) + pow(d,2) )/(2 * d) ;
    double h = sqrt( pow(door.radius,2) - pow(l,2) );
    x1 = (l/d)*(curr_ur3_wx-door.door_hinge_point.point.x) + (h/d)*(curr_ur3_wy-door.door_hinge_point.point.y) + door.door_hinge_point.point.x;
    x2 = (l/d)*(curr_ur3_wx-door.door_hinge_point.point.x) - (h/d)*(curr_ur3_wy-door.door_hinge_point.point.y) + door.door_hinge_point.point.x;
    y1 = (l/d)*(curr_ur3_wy-door.door_hinge_point.point.y) - (h/d)*(curr_ur3_wx-door.door_hinge_point.point.x) + door.door_hinge_point.point.y;
    y2 = (l/d)*(curr_ur3_wy-door.door_hinge_point.point.y) + (h/d)*(curr_ur3_wx-door.door_hinge_point.point.x) + door.door_hinge_point.point.y;

    ROS_INFO("x1 = (%f/%f)*(%f-%f) + (%f/%f)*(%f-%f) + %f",l,d,curr_ur3_wx,door.door_hinge_point.point.x,h,d,curr_ur3_wy,door.door_hinge_point.point.y,door.door_hinge_point.point.x);
    ROS_INFO("x2 = (%f/%f)*(%f-%f) - (%f/%f)*(%f-%f) + %f",l,d,curr_ur3_wx,door.door_hinge_point.point.x,h,d,curr_ur3_wy,door.door_hinge_point.point.y,door.door_hinge_point.point.x);
    ROS_INFO("y1 = (%f/%f)*(%f-%f) - (%f/%f)*(%f-%f) + %f",l,d,curr_ur3_wy,door.door_hinge_point.point.y,h,d,curr_ur3_wx,door.door_hinge_point.point.x,door.door_hinge_point.point.y);
    ROS_INFO("y2 = (%f/%f)*(%f-%f) + (%f/%f)*(%f-%f) + %f",l,d,curr_ur3_wy,door.door_hinge_point.point.y,h,d,curr_ur3_wx,door.door_hinge_point.point.x,door.door_hinge_point.point.y);

    unsigned int idx_check1,idx_check2;
    costmap->worldToMap(x1,y1,mx,my);
    idx_check1 = costmap->getIndex(mx,my);
    costmap->worldToMap(x2,y2,mx,my);
    idx_check2 = costmap->getIndex(mx,my);
    if(!in_first_room(idx_check1) && !in_first_room(idx_check2) && !in_first_room(idx))
    {
        reachable_door_angles_a.first = 4;
        reachable_door_angles_a.second = reachable_door_angles_i;
        reachable_door_angles.push_back(reachable_door_angles_a);
        return reachable_door_angles;
    }

    if(!in_first_room(idx_check1))
    {
        x1 = door.door_handle_point.point.x;
        y1 = door.door_handle_point.point.y;
    }
    
    if(!in_first_room(idx_check2))
    {
        x2 = door.door_handle_point.point.x;
        y2 = door.door_handle_point.point.y;
    }
    double x_small,y_small;//coordinates of smallest reachable door angle
    double x_large,y_large;//coordinates of largest reachable door angle
    ROS_INFO_STREAM("(x1,y1) = ("<<x1<<","<<y1<<")"<<" & (x2,y2) = ("<<x2<<","<<y2<<") & d = "<<d);
    // ROS_INFO_STREAM("atan(x1,y1) = "<<atan((y1-door.door_hinge_point.point.y)/(x1-door.door_hinge_point.point.x))*180/M_PI<<" & atan(x2,y2) = "<<atan((y2-door.door_hinge_point.point.y)/(x2-door.door_hinge_point.point.x))*180/M_PI);
    // if( atan((y1-door.door_hinge_point.point.y)/(x1-door.door_hinge_point.point.x))*180/M_PI <= 
    //     atan((y2-door.door_hinge_point.point.y)/(x2-door.door_hinge_point.point.x))*180/M_PI )
    // {
    //     x_small = x1; x_large = x2;
    //     y_small = y1; y_large = y2;
    // }
    // else
    // {
    //     x_small = x2; x_large = x1;
    //     y_small = y2; y_large = y1;
    // }

    // if(atan((y_large-door.door_hinge_point.point.y)/(x_large-door.door_hinge_point.point.x))*180/M_PI > door.max_door_angle_degree)
    // {
    //     ROS_INFO_STREAM(" Greater than door.max_door_angle_degree ; x_small = "<<x_small<<" & y_small = "<<y_small<<" & x_large = "<<x_large<<" & y_large = "<<y_large);
    //     x_large = door.door_hinge_point.point.x + door.length*cos((90+door.max_door_angle_degree)*M_PI/180) ;
    //     y_large = door.door_hinge_point.point.y + door.length*sin((90+door.max_door_angle_degree)*M_PI/180) ;
    // }

    // if(atan((y_small-door.door_hinge_point.point.y)/(x_small-door.door_hinge_point.point.x))*180/M_PI < 0)
    // {
    //     ROS_INFO_STREAM(" Less than 0 degree door angle ; x_small = "<<x_small<<" & y_small = "<<y_small<<" & x_large = "<<x_large<<" & y_large = "<<y_large);
    //     x_small = door.door_handle_point.point.x;
    //     y_small = door.door_handle_point.point.y;
    // }
    // ROS_INFO_STREAM("x_small = "<<x_small<<" & y_small = "<<y_small<<" & x_large = "<<x_large<<" & y_large = "<<y_large);
    // int srda, lrda;// smallest reachable door angle & largest reachable door angle in degrees
    // double d_small = sqrt(pow((door.door_handle_point.point.x-x_small),2) + pow((door.door_handle_point.point.y-y_small),2));
    // double d_large = sqrt(pow((door.door_handle_point.point.x-x_large),2) + pow((door.door_handle_point.point.y-y_large),2));
    // srda = int(ceil( acos( 1 - ( (pow(d_small,2)) / (2*pow(door.length,2)) ) ) * 180/M_PI ));
    // lrda = int(floor(acos( 1 - ( (pow(d_large,2)) / (2*pow(door.length,2)) ) ) * 180/M_PI ));
    unsigned int door_angle1, door_angle2;// smallest reachable door angle & largest reachable door angle in degrees
    double d1 = sqrt(pow((door.door_handle_point.point.x-x1),2) + pow((door.door_handle_point.point.y-y1),2));
    double d2 = sqrt(pow((door.door_handle_point.point.x-x2),2) + pow((door.door_handle_point.point.y-y2),2));
    door_angle1 = int(ceil( acos( 1 - ( (pow(d1,2)) / (2*pow(door.radius,2)) ) ) * 180/M_PI ));
    door_angle2 = int(floor(acos( 1 - ( (pow(d2,2)) / (2*pow(door.radius,2)) ) ) * 180/M_PI ));
    unsigned int smallest_reachable_door_angle, largest_reachable_door_angle;
    if(door_angle1 <= door_angle2)
    {
        smallest_reachable_door_angle = door_angle1;
        largest_reachable_door_angle = door_angle2;
    }
    else
    {
        smallest_reachable_door_angle = door_angle2;
        largest_reachable_door_angle = door_angle1;
    }
    if(smallest_reachable_door_angle > door.max_door_angle_degree)
        smallest_reachable_door_angle = door.max_door_angle_degree;
    if(largest_reachable_door_angle > door.max_door_angle_degree)
        largest_reachable_door_angle = door.max_door_angle_degree;

    ROS_INFO_STREAM("For ("<<curr_ur3_wx<<","<<curr_ur3_wy<<") smallest_reachable_door_angle = "<<smallest_reachable_door_angle<<" & largest_reachable_door_angle = "<<largest_reachable_door_angle);
    
    // collision check --->   
    
    vector<pair<double,double> > footprint = compute_robot_footprint(curr_wx,curr_wy,theta);//0th index is top_left corner, 1st index is top_right, 2nd index is bottom left, 3rd is bottom right
    vector<unsigned int> a_1_door_angles,a_2_door_angles,a_3_door_angles;
    unsigned int angle;
    for(angle=smallest_reachable_door_angle;angle<=largest_reachable_door_angle;angle++)
    {
        if(door_collision_with_robot(footprint,angle))
        {
            ROS_INFO_STREAM("door_collision_with_robot() returned true for angle = "<<angle);
            continue;
        }
        if(in_first_room(idx))
        {
            if(!inside_line_of_door(idx,angle))
                a_1_door_angles.push_back(angle);
            else
                a_2_door_angles.push_back(angle);
        }
        else
            a_3_door_angles.push_back(angle);
    }
    if(a_3_door_angles.size()>0)
    {
        reachable_door_angles_i = a_3_door_angles;
        reachable_door_angles_a.first = 3;
        reachable_door_angles_a.second = reachable_door_angles_i;
        reachable_door_angles.push_back(reachable_door_angles_a);
        
    }
    else
    {
        if(a_1_door_angles.size()==0 && a_2_door_angles.size()==0)//for faltu edge case
        {
            if(in_first_room(idx))
                reachable_door_angles_a.first = 0;
            else
                reachable_door_angles_a.first = 4;
            reachable_door_angles_a.second = reachable_door_angles_i;
            reachable_door_angles.push_back(reachable_door_angles_a);
        }
        else if(a_1_door_angles.size()==0)
        {
            reachable_door_angles_i = a_2_door_angles;
            reachable_door_angles_a.first = 2;
            reachable_door_angles_a.second = reachable_door_angles_i;
            reachable_door_angles.push_back(reachable_door_angles_a);
            
        }
        else if(a_2_door_angles.size()==0)
        {
            reachable_door_angles_i = a_1_door_angles;
            reachable_door_angles_a.first = 1;
            reachable_door_angles_a.second = reachable_door_angles_i;
            reachable_door_angles.push_back(reachable_door_angles_a);
            
        }
        else
        {
            reachable_door_angles_i = a_1_door_angles;
            reachable_door_angles_a.first = 1;
            reachable_door_angles_a.second = reachable_door_angles_i;
            reachable_door_angles.push_back(reachable_door_angles_a);

            reachable_door_angles_i = a_2_door_angles;
            reachable_door_angles_a.first = 2;
            reachable_door_angles_a.second = reachable_door_angles_i;
            reachable_door_angles.push_back(reachable_door_angles_a);
            
        }
    }
    for(pair<unsigned int , vector<unsigned int> > i:reachable_door_angles)
    {
        ROS_INFO_STREAM("For Current ur3 pose : ("<<curr_ur3_wx<<","<<curr_ur3_wy<<") ; a = "<<i.first<<" & rda_range = [");
        for(unsigned int i_rda:i.second)
            ROS_INFO_STREAM(i_rda);
    }
    ROS_INFO_STREAM("]");
    return reachable_door_angles;

    // unsigned int a = compute_a(idx);
    // if(a==1)
    // {
    //     unsigned int angle;
    //     for(angle=smallest_reachable_door_angle;angle<=largest_reachable_door_angle;angle++)
    //     {
    //         ROS_INFO_STREAM("a==1 case");
    //         if(door_collision_with_robot(footprint,angle))
    //         {
    //             ROS_INFO_STREAM("largest_reachable_door_angle = "<<angle<<" from "<<largest_reachable_door_angle);
    //             largest_reachable_door_angle = angle-1;
    //             break;
    //         }
    //     }
    // }
    // else if(a==2 || a==3)
    // {
    //     unsigned int angle;
    //     for(angle=largest_reachable_door_angle;angle>=smallest_reachable_door_angle && angle<=largest_reachable_door_angle;angle--)
    //     {
    //         // ROS_INFO_STREAM("x = "<<door.door_hinge_point.point.x<<" + "<<door.length<<" * "<<cos((90+angle)*M_PI/180));
    //         // ROS_INFO_STREAM("y = "<<door.door_hinge_point.point.y<<" + "<<door.length<<" * "<<sin((90+angle)*M_PI/180));
    //         // ROS_INFO_STREAM("x = "<<x<<" & y = "<<y<<" for angle = "<<angle);
    //         ROS_INFO_STREAM("a==2 || a==3 case");
    //         if(door_collision_with_robot(footprint,angle))
    //         {
    //             ROS_INFO_STREAM("smallest_reachable_door_angle = "<<angle<<" from "<<smallest_reachable_door_angle);
    //             smallest_reachable_door_angle = angle+1;
    //             break;
    //         }
    //     }
    // }

    
    // reachable_door_angles.first = smallest_reachable_door_angle;
    // reachable_door_angles.second = largest_reachable_door_angle;
    // ROS_INFO_STREAM("reachable_door_angles.first = "<<reachable_door_angles.first<<" & reachable_door_angles.second = "<<reachable_door_angles.second);
    // return reachable_door_angles;

};
double ARAStar::compute_heading_angle(unsigned int s1_idx, unsigned int s2_idx)//computed heading angle as atan(slope of line joining s1_idx & pred[s1_idx])
{
    unsigned int pred_idx;
    if(pred[s1_idx]==0)
        pred_idx = s2_idx;
    else
        pred_idx = pred[s1_idx];

    unsigned int mx,my;
    costmap->indexToCells(pred_idx,mx,my);
    double pred_wx,pred_wy;
    costmap->mapToWorld(mx,my,pred_wx,pred_wy);

    costmap->indexToCells(s1_idx,mx,my);
    double curr_wx,curr_wy;
    costmap->mapToWorld(mx,my,curr_wx,curr_wy);

    return atan((curr_wy-pred_wy)/(curr_wx-pred_wx));
}
bool ARAStar::is_action_feasible(State s,unsigned int succ_idx,vector<unsigned int> succ_rda)
{
    vector<pair<unsigned int , vector<unsigned int> > > s_idx_rda = compute_reachable_door_angles(s.idx,compute_heading_angle(s.idx,succ_idx));
    vector<unsigned int> s_rda;//reachable door angles from state s
    //pair<unsigned int,unsigned int> succ_rda ---> reachable door angles from state succ
    for(pair<unsigned int , vector<unsigned int> > s_idx_a_rda:s_idx_rda)
    {
        if(s_idx_a_rda.first == s.a)
        {
            s_rda = s_idx_a_rda.second;
            break;
        }
    }
    for(unsigned int s_i:s_rda)
    {
        if(find(succ_rda.begin(),succ_rda.end(),s_i)!=succ_rda.end())
            return true;
    }
    return false;
};
double ARAStar::compute_manipulator_dist(unsigned int idx, int c)
{
    //ROS_INFO_STREAM("In compute_manipulator_dist()");
    unsigned int mx,my;
    costmap->indexToCells(idx,mx,my);
    double curr_wx,curr_wy;
    costmap->mapToWorld(mx,my,curr_wx,curr_wy);

    geometry_msgs::PointStamped base_link_point;
    // base_link_point.header.frame_id = "ur3_base_link";
    // base_link_point.header.stamp = ros::Time();
    // base_link_point.point.x = 0.0;
    // base_link_point.point.y = 0.0;
    // base_link_point.point.z = 0.0;
    base_link_point.header.frame_id = "map";
    base_link_point.header.stamp = ros::Time();
    base_link_point.point.x = curr_wx;
    base_link_point.point.y = curr_wy;
    base_link_point.point.z = 0.0;

    // geometry_msgs::PointStamped ur3_base_link_point;
    // trydoor.door_handle_point.point.y
    // {
    //     ros::Time now = ros::Time::now();
    //     //ROS_INFO_STREAM("Pre waitforTransform()");
    //     listener.waitForTransform(base_link_point.header.frame_id, "map", now, ros::Duration(3.0));
    //     //ROS_INFO_STREAM("Post waitforTransform()");
    //     listener.transformPoint("map",base_link_point,ur3_base_link_point);
    // }
    // catch(tf::TransformException ex)
    // {
    //     ROS_INFO_STREAM(" Tranform error in compute_manipulator_dist() : "<<ex.what());
    // }
    // if(c==0)//door handle
    //     return sqrt(pow((door.door_handle_point.point.x - ur3_base_link_point.point.x),2) + pow((door.door_handle_point.point.y - ur3_base_link_point.point.y),2));
    // else//door hinge
    //     return sqrt(pow((door.door_hinge_point.point.x - ur3_base_link_point.point.x),2) + pow((door.door_hinge_point.point.y - ur3_base_link_point.point.y),2));
    if(c==0)//door handle
        return sqrt(pow((door.door_handle_point.point.x - base_link_point.point.x),2) + pow((door.door_handle_point.point.y - base_link_point.point.y),2));
    else//door hinge
        return sqrt(pow((door.door_hinge_point.point.x - base_link_point.point.x),2) + pow((door.door_hinge_point.point.y - base_link_point.point.y),2));
};
bool ARAStar::inside_line_of_door(unsigned int idx, unsigned int door_angle)
{
    //ROS_INFO_STREAM("In inside_line_of_door()");
    // if(compute_manipulator_dist(idx,1) > door.length)
    //     return false;
    // else 
    // {
    if(in_first_room(idx))
    {
        unsigned int mx,my;
        costmap->indexToCells(idx,mx,my);
        double curr_wx,curr_wy;
        costmap->mapToWorld(mx,my,curr_wx,curr_wy);
        //ROS_INFO_STREAM("For idx = "<<idx<<" & angle = "<<door_angle<<" ; check = "<<(atan((curr_wy-door.door_hinge_point.point.y)/(curr_wx-door.door_hinge_point.point.x))*180/M_PI)+90);
        if( (atan((curr_wy-door.door_hinge_point.point.y)/(curr_wx-door.door_hinge_point.point.x))*180/M_PI)+90 >= door_angle) //using base_link here and not ur3_base_link for convenience
            return false;
        else 
            return true;
    }
    else
        return false;
};
bool ARAStar::in_first_room(unsigned int idx)
{
    //ROS_INFO_STREAM("In in_first_room()");
    // have to use base_link or ur3_base_link ?? For now using base_link
    unsigned int mx,my;
    costmap->indexToCells(idx,mx,my);
    double curr_wx,curr_wy;
    costmap->mapToWorld(mx,my,curr_wx,curr_wy);
    double d = (curr_wx - door.door_hinge_point.point.x)*(door.door_handle_point.point.y - door.door_hinge_point.point.y) - (curr_wy - door.door_hinge_point.point.y)*(door.door_handle_point.point.x - door.door_hinge_point.point.x);
    if( d>0 )
        return false;
    else
        return true;
};
vector<pair<unsigned int , vector<unsigned int> > > ARAStar::compute_a(unsigned int idx,double theta)
{
    //ROS_INFO_STREAM("In compute_a()");
    return compute_reachable_door_angles(idx,theta);
    
    // double man_dist = compute_manipulator_dist(idx,0); 
    // //ROS_INFO_STREAM("man_dist = "<<man_dist<<" for idx = "<<idx);
    // unsigned int a;
    // if(man_dist > reach && in_first_room(idx))
    //     a=0;
    // else if(man_dist <= reach && in_first_room(idx) && !inside_line_of_door(idx))
    //     a=1;
    // else if(in_first_room(idx) && inside_line_of_door(idx))
    //     a=2;
    // else if(man_dist <= reach && !in_first_room(idx))
    //     a=3;
    // else if(man_dist > reach && !in_first_room(idx))
    //     a=4;
    
    // return a;
};
double ARAStar::cost(State s, State succ)
{
    double c_action, c_map, c_door=0.0;   
    //ROS_INFO_STREAM("succ.idx = "<<succ.idx); 
    //c_action computation
    if((s.idx - nx == succ.idx) || (s.idx + nx == succ.idx) || (s.idx - 1 == succ.idx) || (s.idx + 1  == succ.idx))
        c_action = 1.0;
    else if((s.idx - nx - 1 == succ.idx) || (s.idx - nx + 1 == succ.idx) || (s.idx + nx -1 == succ.idx) || (s.idx + nx + 1 == succ.idx))
        c_action = 1.4;
    else 
        c_action = INT16_MAX;

    //c_map computation
    c_map = map_[succ.idx];
    //ROS_INFO_STREAM("succ.idx = "<<succ.idx<<" & succ.a = "<<succ.a<<" & s.idx = "<<s.idx<<" & s.a = "<<s.a); 
    //c_door computation
    if( (s.a>=1 && s.a<=3) && (succ.a >=1 && succ.a<=3) )
    {
        c_door = INT16_MAX;
        // c_door = K*pow((d - d_min),2)
        double K, d, d_min;
        K=1.0; // positive coefficient
        d_min = 0.05; // 0.05m : based on reachability data

        // calculate d:dist from base of manipulator to door handle 
        vector<pair<unsigned int , vector<unsigned int> > > s_idx_rda = compute_reachable_door_angles(s.idx,compute_heading_angle(s.idx,succ.idx));
        vector<unsigned int> s_rda;//reachable door angles from state s
        //ROS_INFO_STREAM();
        for(pair<unsigned int , vector<unsigned int> > s_idx_a_rda:s_idx_rda)
        {
            //ROS_INFO_STREAM("s_idx_a_rda.second.size() = "<<s_idx_a_rda.second.size());
            if(s_idx_a_rda.first == s.a)
            {
                s_rda = s_idx_a_rda.second;
                break;
            }
        }
        
        //ROS_INFO_STREAM("s_rda.size() = "<<s_rda.size());
        unsigned int min_cost_door_angle;
        double x_door_handle,y_door_handle,c_door_angle=0.0;
        unsigned int mx,my;
        costmap->indexToCells(s.idx,mx,my);
        double curr_wx,curr_wy;
        costmap->mapToWorld(mx,my,curr_wx,curr_wy);
        //transformed from base_link's coordinates to ur3_base_link's coordinates
        double curr_ur3_wx,curr_ur3_wy;
        curr_ur3_wx = curr_wx + transform.getOrigin().x();
        curr_ur3_wy = curr_wy + transform.getOrigin().y();

        for(unsigned int door_angle:s_rda)
        {
            x_door_handle = door.door_hinge_point.point.x + (door.radius) * cos((90+door_angle)*M_PI/180);
            y_door_handle = door.door_hinge_point.point.y + (door.radius) * sin((90+door_angle)*M_PI/180);
            d = hypot( (curr_ur3_wx-x_door_handle) , (curr_ur3_wy-y_door_handle) );//dist betn door handle and ur3_base_link when door is at door_angle
            //ROS_INFO_STREAM("d = "<<d<<" for door_angle = "<<door_angle);
            c_door_angle = K*pow((d - d_min),2);
            //ROS_INFO_STREAM("c_door = "<<c_door_angle<<" for door_angle = "<<door_angle);
            if(c_door_angle<c_door)
            {
                c_door = c_door_angle;
                min_cost_door_angle = door_angle;
            }
        }
        //ROS_INFO_STREAM("c_door(final wala) = "<<c_door<<" for succ.idx = "<<succ.idx);
        idx_to_door_angle[s.idx] = min_cost_door_angle;
    }
    
    return c_action + c_map + c_door;
};
vector<unsigned int> ARAStar::compute_successors_idx_set(State s)
{
    int noac = 8;// noac : number of adjacent cells (eight point connectivity metric / four point connectivity metric) 
    vector<unsigned int> successors(noac,-1);
    if(noac == 8 )//8 point connectivity
    {
        int zi=-1,zj=-1,k=0;
        unsigned int adj_idx;
        for(int i=1;i<=noac+1;i++)
        {  
            if(zi==-1)  
                adj_idx = s.idx - nx + zj;
            else if(zi==0)
                adj_idx = s.idx + zj; 
            else if(zi==1)
                adj_idx = s.idx + nx + zj;

            if(adj_idx != s.idx)
                successors[k++] = adj_idx;

            if(i%3==0)
            {    zi++; zj=-1;}
            else
                zj++;
        }
    }
    if(noac == 4)// 4 point connectivity
    {
        int k=0;
        successors[k++] = s.idx - nx;
        successors[k++] = s.idx - 1;
        successors[k++] = s.idx + 1 ;
        successors[k++] = s.idx + nx;
    }
    return successors;
};
bool ARAStar::is_occupied(unsigned int succ_idx, unsigned int s_idx)
{
    unsigned int mx,my,idx;
    costmap->indexToCells(succ_idx,mx,my);
    double wx,wy;
    costmap->mapToWorld(mx,my,wx,wy);
    vector<pair<double,double> > footprint = compute_robot_footprint(wx,wy,compute_heading_angle(succ_idx,s_idx));
    vector<costmap_2d::MapLocation> footprint_mx_my;
    costmap_2d::MapLocation footprint_map_coordinate;
    // ROS_INFO_STREAM("footprint_mx_my = [");
    for(pair<double,double> footprint_world_coordinate:footprint)
    {
        costmap->worldToMap(footprint_world_coordinate.first,footprint_world_coordinate.second,mx,my);
        footprint_map_coordinate.x = mx;
        footprint_map_coordinate.y = my;
        // ROS_INFO_STREAM("{"<<mx<<","<<my<<"}");
        footprint_mx_my.push_back(footprint_map_coordinate);
    }
    // ROS_INFO_STREAM("]");
    vector<costmap_2d::MapLocation> footprint_cells;
    costmap->convexFillCells(footprint_mx_my,footprint_cells);
    ROS_INFO_STREAM("footprint_cells.size() = "<<footprint_cells.size());
    //ROS_INFO_STREAM("footprint_cells = [");
    for(costmap_2d::MapLocation map_cell:footprint_cells)
    {
        idx = costmap->getIndex(map_cell.x,map_cell.y);
        //ROS_INFO_STREAM(idx);
        if(map_[idx]>128)
        {
            ROS_INFO_STREAM("Returning true for idx = "<<idx);
            return true;
        }
    }
    //ROS_INFO_STREAM("]");
    return false;
};
void ARAStar::compute_cartesian_door_handle_path()
{
    nav_msgs::Path cartesian_door_angle_path;
    cartesian_door_angle_path.header.frame_id = "map";
    geometry_msgs::PoseStamped door_handle_pose;
    door_handle_pose.header.stamp = ros::Time::now();
    door_handle_pose.header.frame_id = "map";
    
    door_handle_pose.pose.position.z = 0.8;
    door_handle_pose.pose.orientation.x = 0.0;
    door_handle_pose.pose.orientation.y = 0.0;
    door_handle_pose.pose.orientation.z = 0.0;
    door_handle_pose.pose.orientation.w = 1.0;
    for(unsigned int door_angle:door_angle_path)
    {
        door_handle_pose.pose.position.x = door.door_hinge_point.point.x + door.radius * cos((90+door_angle)*M_PI/180);
        door_handle_pose.pose.position.y = door.door_hinge_point.point.y + door.radius * sin((90+door_angle)*M_PI/180);
        ROS_INFO_STREAM("(x,y) = ("<<door_handle_pose.pose.position.x<<","<<door_handle_pose.pose.position.y<<") for door_angle = "<<door_angle);
        cartesian_door_angle_path.poses.push_back(door_handle_pose);
    }
    msg.door_handle_pose = cartesian_door_angle_path;
    // door_handle_path_pub.publish(cartesian_door_angle_path);
};
vector<unsigned int> ARAStar::compute_idx_path()
{
    vector<unsigned int> idx_path;
    idx_path.push_back(goal.idx);
    unsigned int crawl = goal.idx;
    while(pred[crawl]!=0)
    {
        idx_path.push_back(pred[crawl]);

        //door_angle_path computation -->
        if(idx_to_door_angle.find(crawl) != idx_to_door_angle.end())
            door_angle_path.push_back(idx_to_door_angle[crawl]);

        if(crawl == start.idx)
            break;
        crawl=pred[crawl];
    }
    reverse(idx_path.begin(),idx_path.end());
    reverse(door_angle_path.begin(),door_angle_path.end());
    compute_cartesian_door_handle_path();
    return idx_path;
    
};
bool ARAStar::in_closed(State* succ)
{
    for(auto closed_i:closed)
    {
        if(closed_i.idx==succ->idx)
            return true;
    }
    return false;
};
unsigned int ARAStar::fvalue(State s)
{
    return s.g + s.epsilon * s.h;
};
void ARAStar::improvePath()
{
    State s; bool found = false;
    //ROS_INFO_STREAM(" fvalue(goal) = "<<fvalue(goal)<<" & goal.idx = "<<goal.idx<<" & fvalue(*(open.begin())) = "<<fvalue(*(open.begin()))<<" & (*(open.begin())).idx = "<<(*(open.begin())).idx<<" & open.size() = "<<open.size());
    unsigned int k=1; vector<pair<unsigned int , vector<unsigned int> > > succ_a;
    while( fvalue(goal) > fvalue(*(open.begin())) )
    {
        s = *(open.begin()); open.erase(*(open.begin()));
        // ROS_INFO_STREAM(" State s.idx = "<<s.idx<<" & goal.idx = "<<goal.idx<<" & fvalue(s) = "<<fvalue(s)<<" & fvalue(goal) = "<<fvalue(goal)<< " & epsilon = "<<epsilon<<" open.size() = "<<open.size());
        ROS_INFO_STREAM("State s.idx = "<<s.idx<<" ; in_first_room(s.idx) = "<<in_first_room(s.idx)<<" & s.a = "<<s.a<<" & k = "<<k++<<" at ros::Time::now().toSec() = "<< ros::Time::now().toSec()<<" & open.size() = "<<open.size());
        closed.insert(s);
        vector<unsigned int> successors;
        //ROS_INFO_STREAM(" PRE COMPUTE_SUCCESSORS_IDX_SET() = "<< ros::Time::now().toSec());
        successors = compute_successors_idx_set(s);
        //ROS_INFO_STREAM(" POST COMPUTE_SUCCESSORS_IDX_SET() = "<< ros::Time::now().toSec());
        for(unsigned int succ_idx:successors)
        {
            // if(map_[succ_idx]>128)
            //     continue;
            if(is_occupied(succ_idx,s.idx))
                continue;
            //ROS_INFO_STREAM(" PRE COMPUTE_A() = "<< ros::Time::now().toSec());
            succ_a = compute_a(succ_idx,compute_heading_angle(succ_idx,s.idx));//ROS_INFO_STREAM("succ_a.size() = "<< succ_a.size());
            for(pair<unsigned int , vector<unsigned int> > successor_a : succ_a)
            {
                if(successor_a.second.size()>0)
                    ROS_INFO_STREAM("{"<<successor_a.first<<", {"<<successor_a.second[0]<<" , "<<successor_a.second[successor_a.second.size()-1]<<" and size = "<<successor_a.second.size()<<"} }");
                if( (s.a>=1 && s.a<=3) && (successor_a.first >= 1 && successor_a.first <=3) )//(s.a>=1 && s.a<=3) && 
                {//ROS_INFO_STREAM(" PRE IS_ACTION_FEASIBLE() = "<< ros::Time::now().toSec());
                    if(!is_action_feasible(s,succ_idx,successor_a.second))
                    {
                        ROS_INFO_STREAM("ACTION NOT FEASIBLE FROM s.idx = "<<s.idx<<" to succ_idx = "<<succ_idx);
                        continue;
                    }
                    ROS_INFO_STREAM("ACTION IS FEASIBLE!!! FROM s.idx = "<<s.idx<<" to succ_idx = "<<succ_idx);
                    //compute_reachable_door_angles(succ_idx);
                }
                //ROS_INFO_STREAM("succ_idx = "<<succ_idx);
                //ROS_INFO_STREAM("visited[succ_idx] = "<<visited[succ_idx]);
                if(visited[succ_idx]==NULL)
                {
                    //ROS_INFO_STREAM(" PRE HEURISTIC() = "<< ros::Time::now().toSec());
                    State* successor = new State(succ_idx,double(INT16_MAX),heuristic(succ_idx,compute_heading_angle(succ_idx,s.idx),successor_a.first),epsilon,successor_a.first);//ROS_INFO_STREAM(" POST HEURISTIC() = "<< ros::Time::now().toSec());
                    if(succ_idx == goal.idx)
                        visited[succ_idx]=&goal;
                    else
                    {
                        visited[succ_idx]=successor;
                        //ROS_INFO_STREAM("if->else ke andar visited[succ_idx]->idx = "<<(visited[succ_idx])->idx);
                    }
                    //visited[succ_idx]=&successor;
                    //ROS_INFO_STREAM("if ke andar visited[succ_idx]->idx = "<<(visited[succ_idx])->idx);
                    succ = visited[succ_idx];
                    //ROS_INFO_STREAM("if ke andar succ->idx = "<<succ->idx);
                    pred[succ_idx] = s.idx;
                    
                }
                //ROS_INFO_STREAM("if ke bahar visited[succ_idx]->idx = "<<(visited[succ_idx])->idx);
                succ = visited[succ_idx];
                //ROS_INFO_STREAM("if ke bahar succ->idx = "<<succ->idx);
                
                if((*succ).g > s.g + cost(s,(*succ)))
                {
                    (*succ).g = s.g + cost(s,(*succ));
                    //ROS_INFO_STREAM(" PRE IN_CLOSED() = "<< ros::Time::now().toSec()<<" FOR CLOSED.SIZE() = "<<closed.size());
                    if(!in_closed(succ))   
                        open.insert((*succ));
                    else
                        incons.insert((*succ));
                    //ROS_INFO_STREAM(" POST IN_CLOSED() = "<< ros::Time::now().toSec()<<" FOR CLOSED.SIZE() = "<<closed.size());
                }
                // if(succ_idx == goal.idx)
                // {
                //     ROS_INFO_STREAM(" succ_idx("<<succ_idx<<") == goal("<<goal.idx<<")");
                //     found=true;
                //     break;
                // }
            }
        }
        // if(found==true){
        //     break;}
    }
};
vector<geometry_msgs::PoseStamped> ARAStar::search()
{
    //ROS_INFO_STREAM(" ARAStar::search() first line ; fvalue(start) = "<<fvalue(start)<<" where start.h = "<<start.h<<" and start.g = "<<start.g<<" and epsilon = "<<epsilon);
    open.insert(start);
    improvePath();
    //publish current epsilon-suboptimal solution
    while( epsilon > 1 )
    {
        epsilon -= 0.2;
        open.insert(incons.begin() , incons.end());
        closed.clear();
        improvePath();
        //publish current epsilon-suboptimal solution
    }

    vector<unsigned int> path = compute_idx_path();
    vector<geometry_msgs::PoseStamped> plan;
    ROS_INFO_STREAM(" PATH.SIZE() = "<<path.size());
    ros::Time plan_time = ros::Time::now();
    for(int i=0;i<path.size();i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = "map";
        unsigned int mx1,my1;
        costmap->indexToCells(path[i],mx1,my1);
        double wx,wy;
        costmap->mapToWorld(mx1,my1,wx,wy);
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
    nav_msgs::Path robot_path;
    robot_path.header.frame_id = "map";
    robot_path.poses = plan;
    msg.robot_pose = robot_path;
    robot_door_handle_path_pub.publish(msg);

    ROS_INFO_STREAM("AB DELETION SHURU HOGA");
    ROS_INFO_STREAM("visited.size() = "<<visited.size());
    auto visited_end = visited.end();
    visited_end--;
    ROS_INFO_STREAM("visited_end->first = "<<visited_end->first);
    unsigned int i=1;
    //delete dynamically allocated memory in visited
    //for(i=0;i<visited.size();i++)
    for(auto it=visited.begin();it!=visited.end();it++)
    {
        ROS_INFO_STREAM("it->first = "<<it->first<<" ; i = "<<i++<<" ; it->second = "<<it->second);
        //ROS_INFO_STREAM("(visited[i])->idx = "<<(visited[i])->idx<<" & i = "<<i);
        //ROS_INFO_STREAM("i = "<<i);
        // delete visited[i];
        // visited[i]=NULL;
        // delete it;
        delete it->second;
        it->second = NULL;
    }
    //return compute_idx_path();
    return plan;
};

bool comp::operator()(const State& s1, const State& s2)
{
    ARAStar use_fvalue_func;
    if(use_fvalue_func.fvalue(s1) < use_fvalue_func.fvalue(s2))
        return true;
    else if(use_fvalue_func.fvalue(s1) == use_fvalue_func.fvalue(s2))
    {
        if(s1.idx < s2.idx)
            return true;
        else
            return false;
    }
    else
        return false;
};
