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
    start.h = heuristic(start_idx_oned);
    start.epsilon = epsilon;
    //initialise goal state
    goal.idx = goal_idx_oned;
    goal.g = double(INT16_MAX);
    goal.h = 0;
    goal.epsilon = epsilon;
    unsigned int mx,my;
    costmap->indexToCells(goal_idx_oned,mx,my);
    costmap->mapToWorld(mx,my,goal_wx,goal_wy);
    
    map_ = costmap->getCharMap();
};
double ARAStar::heuristic(int idx)
{
    unsigned int mx,my;
    costmap->indexToCells(idx,mx,my);
    double curr_wx,curr_wy;
    costmap->mapToWorld(mx,my,curr_wx,curr_wy);
    double world_dist = sqrt(pow((goal_wx-curr_wx),2) + pow((goal_wy-curr_wy),2)); //euclidian distance from curr(wx,wy) to goal(wx,wy)
    return costmap->cellDistance(world_dist); //euclidian distance in terms of cell distance
};
double ARAStar::cost(State s, State succ)
{
    double c_action, c_map;
    
    //c_action computation
    if((s.idx - nx == succ.idx) || (s.idx + nx == succ.idx) || (s.idx - 1 == succ.idx) || (s.idx + 1  == succ.idx))
        c_action = 1.0;
    else if((s.idx - nx - 1 == succ.idx) || (s.idx - nx + 1 == succ.idx) || (s.idx + nx -1 == succ.idx) || (s.idx + nx + 1 == succ.idx))
        c_action = 1.4;
    else 
        c_action = INT16_MAX;

    //c_map computation
    c_map = map_[succ.idx];
    
    return c_action + c_map;
};
vector<int> ARAStar::compute_successors_idx_set(State s)
{
    int noac = 8;// noac : number of adjacent cells (eight point connectivity metric) 
    vector<int> successors(noac,-1);// noac : number of adjacent cells (eight point connectivity metric)
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
    return successors;
};
vector<int> ARAStar::compute_idx_path()
{
    vector<int> idx_path;
    idx_path.push_back(goal.idx);
    int crawl = goal.idx;
    while(pred[crawl]!=0)
    {
        idx_path.push_back(pred[crawl]);
        if(crawl == start.idx)
            break;
        crawl=pred[crawl];
    }
    reverse(idx_path.begin(),idx_path.end());
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

    while( fvalue(goal) > fvalue(*(open.begin())) )
    {
        s = *(open.begin()); open.erase(*(open.begin()));
        //ROS_INFO_STREAM(" State s.idx = "<<s.idx<<" & goal.idx = "<<goal.idx<<" & fvalue(s) = "<<fvalue(s)<<" & fvalue(goal) = "<<fvalue(goal)<< " & epsilon = "<<epsilon<<" open.size() = "<<open.size());
        closed.insert(s);
        vector<int> successors;
        successors = compute_successors_idx_set(s);
        for(int succ_idx:successors)
        {
            if(map_[succ_idx]>128)
                continue;
            
            if(visited[succ_idx]==NULL)
            {
                State successor(succ_idx,double(INT16_MAX),heuristic(succ_idx),epsilon);
                if(succ_idx == goal.idx)
                    visited[succ_idx]=&goal;
                else
                    visited[succ_idx]=&successor;
                //visited[succ_idx]=&successor;
                succ = visited[succ_idx];
                pred[succ_idx] = s.idx;
                
            }
            
            if((*succ).g > s.g + cost(s,(*succ)))
            {
                (*succ).g = s.g + cost(s,(*succ));
                if(!in_closed(succ))   
                    open.insert((*succ));
                else
                    incons.insert((*succ));
            }
            // if(succ_idx == goal.idx)
            // {
            //     ROS_INFO_STREAM(" succ_idx("<<succ_idx<<") == goal("<<goal.idx<<")");
            //     found=true;
            //     break;
            // }
        }
        // if(found==true){
        //     break;}
        // ROS_INFO_STREAM(" openi print: = ");
    }
};
vector<int> ARAStar::search()
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
    return compute_idx_path();
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
