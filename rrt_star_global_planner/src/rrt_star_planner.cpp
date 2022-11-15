/*
  Copyright 2021 - Rafael Barreto
*/

#include <pluginlib/class_list_macros.h>

#include "rrt_star_planner.hpp"

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_star_global_planner::RRTStarPlanner, nav_core::BaseGlobalPlanner)

namespace rrt_star_global_planner {

    RRTStarPlanner::RRTStarPlanner() : costmap_(nullptr), initialized_(false) {}

    RRTStarPlanner::RRTStarPlanner(std::string name,
                                   costmap_2d::Costmap2DROS* costmap_ros) : costmap_(nullptr), initialized_(false) {
        // initialize the planner
        initialize(name, costmap_ros);
    }

    RRTStarPlanner::RRTStarPlanner(std::string name,
                                   costmap_2d::Costmap2D* costmap,
                                   std::string global_frame) : costmap_(nullptr), initialized_(false) {
        // initialize the planner
        initialize(name, costmap, global_frame);
    }

    void RRTStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
        initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
    }

    void RRTStarPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame) {
        if(!initialized_){
            costmap_ = costmap;
            width = costmap_->getSizeInCellsX();
            height = costmap_->getSizeInCellsY();
            map_size = width * height;
            OGM.resize(map_size);

        }
        if (!initialized_) {
            global_frame_ = global_frame;

            ros::NodeHandle private_nh("~/" + name);
            private_nh.param("goal_tolerance", goal_tolerance_, 0.5);
            private_nh.param("radius", radius_, 1.0);
            private_nh.param("epsilon", epsilon_, 0.1);
            private_nh.param("max_num_nodes", max_num_nodes_, 8000);
            private_nh.param("min_num_nodes", min_num_nodes_, 2000);

            // TODO(Rafael) remove hard coding
            if (search_specific_area_) {
                map_width_ = 10.0;
                map_height_ = 10.0;
            } else {
                map_width_ = costmap_->getSizeInMetersX();
                map_height_ = costmap_->getSizeInMetersY();
            }
            plan_pub_ = private_nh.advertise<nav_msgs::Path>("RRTplan", 1);
            ROS_INFO("RRT* Global Planner initialized successfully.");
            initialized_ = true;
        } else {
            ROS_WARN("This planner has already been initialized... doing nothing.");
        }
    }

    bool RRTStarPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                  const geometry_msgs::PoseStamped& goal,
                                  std::vector<geometry_msgs::PoseStamped>& plan) {
        // clear the plan, just in case
        plan.clear();
        bestplan_.clear();
        ROS_INFO("RRT* Global Planner");
        ROS_INFO("Current Position: ( %.2lf, %.2lf)", start.pose.position.x, start.pose.position.y);
        ROS_INFO("GOAL Position: ( %.2lf, %.2lf)", goal.pose.position.x, goal.pose.position.y);

        std::pair<float, float> start_point = {start.pose.position.x, start.pose.position.y};
        std::pair<float, float> goal_point = {goal.pose.position.x, goal.pose.position.y};

        planner_ = std::shared_ptr<RRTStar>(new RRTStar(start_point,
                                                        goal_point,
                                                        costmap_,
                                                        goal_tolerance_,
                                                        radius_,
                                                        epsilon_,
                                                        max_num_nodes_,
                                                        min_num_nodes_,
                                                        map_width_,
                                                        map_height_));

        std::list<std::pair<float, float>> path;

        if (planner_->pathPlanning(path, cost_)) {
            ROS_INFO("RRT* Global Planner: Path found!!!!");
            computeFinalPlan(plan, path);
            return true;
        } else {
            plan.clear();
            return aStar(start,goal,plan);
            //ROS_WARN("The planner failed to find a path, choose other goal position");
            //return false;
        }
    }

    void  RRTStarPlanner::computeFinalPlan(std::vector<geometry_msgs::PoseStamped>& plan,
                                           const std::list<std::pair<float, float>> &path) {
        // clean plan
        plan.clear();
        bestplan_.clear();
        ros::Time plan_time = ros::Time::now();

        // convert points to poses
        for (const auto &point : path) {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = plan_time;
            pose.header.frame_id = global_frame_;
            pose.pose.position.x = point.first;
            pose.pose.position.y = point.second;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            bestplan_.push_back(pose);
        }
        //bestplan_.push_back(goal);
//    for (auto it = bestplan_.begin();  it!=bestplan_.end() ; it++) {
//        ROS_INFO("x=%f, y=%f",it->pose.position.x,it->pose.position.y);
//    }
        CalcSpline(bestplan_,plan);
        //ROS_INFO("cost is %f",cost_);
        //plan.push_back(goal);
        publishPlan(plan);
    }
    void RRTStarPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {

        //create a message for the plan
        nav_msgs::Path gui_path;
        gui_path.poses.resize(path.size());

        gui_path.header.frame_id = global_frame_;
        gui_path.header.stamp = ros::Time::now();

        // Extract the plan in world co-ordinates, we assume the path is all in the same frame
        for (unsigned int i = 0; i < path.size(); i++) {
            gui_path.poses[i] = path[i];
        }

        plan_pub_.publish(gui_path);
    }
    bool RRTStarPlanner::CalcSpline(vector<geometry_msgs::PoseStamped> &bestTraj, vector<geometry_msgs::PoseStamped> &smoothTraj){
        ROS_INFO("Smoothing!!!");
        smoothTraj.clear();
        //int division = 1;
        int order = bestTraj.size()-1;
//        for(int i = 0; i <= order; i++){
//            ROS_INFO("current x = %f, y = %f",bestTraj[division * i].pose.position.x,bestTraj[division * i].pose.position.y);
//        }
        double dt = 1.0000 / (cost_/0.07);
        int seg=21;
        vector<std::pair<double, double>> traj;
        ros::Time plan_time = ros::Time::now();
        if((cost_/0.07)<seg){
            for (int m = 0; m < (cost_/0.07); ++m) {
                pair<double, double> pair;
                pair.first = 0;
                pair.second = 0;
                for (int i = 0; i <= order; i++) {
                    //ROS_INFO("%f",m*dt);
                    //ROS_INFO("current x = %f, y = %f",bestTraj[division * i].pose.position.x,bestTraj[division * i].pose.position.y);
                    pair.first += CalcTerm(order, bestTraj[i], i, m*dt).first;
                    pair.second += CalcTerm(order, bestTraj[i], i, m*dt).second;

                }

                //ROS_INFO("x = %f, y = %fBZ x = %f, y =%f",bestTraj[m].pose.position.x,bestTraj[m].pose.position.y,pair.first,pair.second);
                geometry_msgs::PoseStamped pose;

                pose.header.stamp = plan_time;
                pose.header.frame_id = global_frame_;
                pose.pose.position.x = pair.first;
                pose.pose.position.y = pair.second;
                pose.pose.position.z = 0;
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = 1.0;
                smoothTraj.push_back(pose);
            }
        }else{
            (floor(seg/(cost_/0.07)))*bestTraj.size()-1>2? order = (floor(seg/(cost_/0.07)))*bestTraj.size()-1:2;
            ROS_INFO("order is %d",order);
            for (int m = 0; m < seg; ++m) {
                pair<double, double> pair;
                pair.first = 0;
                pair.second = 0;
                for (int i = 0; i <= order; i++) {
                    //ROS_INFO("%f",m*dt);
                    //ROS_INFO("current x = %f, y = %f",bestTraj[division * i].pose.position.x,bestTraj[division * i].pose.position.y);
                    pair.first += CalcTerm(order, bestTraj[i], i, m*dt).first;
                    pair.second += CalcTerm(order, bestTraj[i], i, m*dt).second;

                }
                geometry_msgs::PoseStamped pose;

                pose.header.stamp = plan_time;
                pose.header.frame_id = global_frame_;
                pose.pose.position.x = pair.first;
                pose.pose.position.y = pair.second;
                pose.pose.position.z = 0;
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = 1.0;
                smoothTraj.push_back(pose);
            }
//            for (int m = seg; m < (cost_/0.07); ++m) {
//                geometry_msgs::PoseStamped pose;
//
//                pose.header.stamp = plan_time;
//                pose.header.frame_id = global_frame_;
//                pose.pose.position.x = bestTraj[m].pose.position.x;
//                pose.pose.position.y = bestTraj[m].pose.position.y;
//                pose.pose.position.z = 0;
//                pose.pose.orientation.x = 0.0;
//                pose.pose.orientation.y = 0.0;
//                pose.pose.orientation.z = 0.0;
//                pose.pose.orientation.w = 1.0;
//                smoothTraj.push_back(pose);
//            }
        }
        return true;
    }
    bool RRTStarPlanner::aStar(const geometry_msgs::PoseStamped& start,
                                   const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
        OGM.clear();
        for (int i = 0; i < width; i++)
        {
            for (int j = 0; j < height; j++)
            {
                unsigned int cost = static_cast<int>(costmap_->getCost(j, i));
                //get_cost << cost << endl;
                //cout << "i:, j:" << cost << endl;

                if (cost < 150)
                    OGM[i * width + j] = true;
                else
                    OGM[i * width + j] = false;

            }
        }
        if(!initialized_){
            ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
            return false;
        }

        ROS_INFO("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y,
                 goal.pose.position.x,goal.pose.position.y);
        double wx = start.pose.position.x;
        double wy = start.pose.position.y;
        unsigned int start_x, start_y;
        costmap_->worldToMap(wx, wy, start_x, start_y);
        int start_index = costmap_->getIndex(start_x, start_y);


        wx = goal.pose.position.x;
        wy = goal.pose.position.y;

        unsigned int goal_x, goal_y;
        costmap_->worldToMap(wx, wy, goal_x, goal_y);
        int goal_index = costmap_->getIndex(goal_x, goal_y);

        vector<float> gCosts(map_size, infinity);
        vector<int> cameFrom(map_size, -1);

        multiset<NodeA> priority_costs;

        gCosts[start_index] = 0;

        NodeA currentNode;
        currentNode.index = start_index;
        currentNode.gCost=0;
        currentNode.hCost= getHeuristic(currentNode.index, goal_index);
        currentNode.cost = gCosts[start_index] + 0;
        priority_costs.insert(currentNode);
        bestplan_.clear();
        plan.clear();

        while(!priority_costs.empty())
        {
            // Take the element from the top
            currentNode = *priority_costs.begin();
            //Delete the element from the top
            priority_costs.erase(priority_costs.begin());
            if (currentNode.index == goal_index){
                break;
            }
            // Get neighbors
            vector<int> neighborIndexes = get_neighbors(currentNode.index);

            for(int i = 0; i < neighborIndexes.size(); i++){
                if(cameFrom[neighborIndexes[i]] == -1){

                    NodeA nextNode;
                    nextNode.index = neighborIndexes[i];
                    unsigned int tx,ty;
                    costmap_->indexToCells(nextNode.index, tx,ty);
                    gCosts[neighborIndexes[i]] = gCosts[currentNode.index] + getMoveCost(currentNode.index, neighborIndexes[i])+0.08*costmap_->getCost(tx,ty);
                    nextNode.gCost = gCosts[neighborIndexes[i]];
                    //nextNode.cost = gCosts[neighborIndexes[i]];    //Dijkstra Algorithm
                    nextNode.cost = gCosts[neighborIndexes[i]] + getHeuristic(neighborIndexes[i], goal_index);    //A* Algorithm
                    cameFrom[neighborIndexes[i]] = currentNode.index;
                    priority_costs.insert(nextNode);
                }
            }
        }

        if(cameFrom[goal_index] == -1){
            cout << "Goal not reachable, failed making a global path." << endl;
            return false;
        }

        if(start_index == goal_index)
            return false;
        //Finding the best path
        vector<int> bestPath;
        currentNode.index = goal_index;
        while(currentNode.index != start_index){
            bestPath.push_back(cameFrom[currentNode.index]);
            currentNode.index = cameFrom[currentNode.index];
        }
        reverse(bestPath.begin(), bestPath.end());

        ros::Time plan_time = ros::Time::now();
        for(int i = 0; i < bestPath.size(); i++){
            unsigned int tmp1, tmp2;
            costmap_->indexToCells(bestPath[i], tmp1, tmp2);
            double x, y;
            costmap_->mapToWorld(tmp1,tmp2, x, y);

            geometry_msgs::PoseStamped pose;
            pose.header.stamp = plan_time;
            pose.header.frame_id = global_frame_;
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = 0.0;

            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;

            bestplan_.push_back(pose);
        }
        bestplan_.push_back(goal);
        CalcSplineA(bestplan_,plan);
        //plan.push_back(goal);
        publishPlan(plan);
        return true;
    }
    bool RRTStarPlanner::CalcSplineA(vector<geometry_msgs::PoseStamped> &bestTraj, vector<geometry_msgs::PoseStamped> &smoothTraj){
        ROS_INFO("Smoothing!!!");
        smoothTraj.clear();
        int division = 3;
        int order = bestTraj.size()/division-1;
//        for(int i = 0; i <= order; i++){
//            ROS_INFO("current x = %f, y = %f",bestTraj[division * i].pose.position.x,bestTraj[division * i].pose.position.y);
//        }
        double dt = 1.0000 / bestTraj.size();
        int seg=22;
        vector<std::pair<double, double>> traj;
        ros::Time plan_time = ros::Time::now();
        if(bestTraj.size()<seg){
            for (int m = 0; m < bestTraj.size(); ++m) {
                pair<double, double> pair;
                pair.first = 0;
                pair.second = 0;
                for (int i = 0; i <= order; i++) {
                    //ROS_INFO("%f",m*dt);
                    //ROS_INFO("current x = %f, y = %f",bestTraj[division * i].pose.position.x,bestTraj[division * i].pose.position.y);
                    pair.first += CalcTerm(order, bestTraj[division * i], i, m*dt).first;
                    pair.second += CalcTerm(order, bestTraj[division * i], i, m*dt).second;

                }

                //ROS_INFO("x = %f, y = %fBZ x = %f, y =%f",bestTraj[m].pose.position.x,bestTraj[m].pose.position.y,pair.first,pair.second);
                geometry_msgs::PoseStamped pose;

                pose.header.stamp = plan_time;
                pose.header.frame_id = global_frame_;
                pose.pose.position.x = pair.first;
                pose.pose.position.y = pair.second;
                pose.pose.position.z = 0;
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = 1.0;
                smoothTraj.push_back(pose);
            }
        }else{
            order = seg/division-1;
            for (int m = 0; m < seg; ++m) {
                pair<double, double> pair;
                pair.first = 0;
                pair.second = 0;
                for (int i = 0; i <= order; i++) {
                    //ROS_INFO("%f",m*dt);
                    //ROS_INFO("current x = %f, y = %f",bestTraj[division * i].pose.position.x,bestTraj[division * i].pose.position.y);
                    pair.first += CalcTerm(order, bestTraj[division * i], i, m*dt).first;
                    pair.second += CalcTerm(order, bestTraj[division * i], i, m*dt).second;

                }
                geometry_msgs::PoseStamped pose;

                pose.header.stamp = plan_time;
                pose.header.frame_id = global_frame_;
                pose.pose.position.x = pair.first;
                pose.pose.position.y = pair.second;
                pose.pose.position.z = 0;
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = 1.0;
                smoothTraj.push_back(pose);
            }
            for (int m = seg; m < bestTraj.size(); ++m) {
                geometry_msgs::PoseStamped pose;

                pose.header.stamp = plan_time;
                pose.header.frame_id = global_frame_;
                pose.pose.position.x = bestTraj[m].pose.position.x;
                pose.pose.position.y = bestTraj[m].pose.position.y;
                pose.pose.position.z = 0;
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = 1.0;
                smoothTraj.push_back(pose);
            }
        }
        return true;
    }
    double RRTStarPlanner::getMoveCost(int firstIndex, int secondIndex)
    {
        unsigned int tmp1, tmp2;
        costmap_->indexToCells(firstIndex, tmp1, tmp2);
        int firstXCord = tmp1,firstYCord = tmp2;
        costmap_->indexToCells(secondIndex, tmp1, tmp2);
        int secondXCord = tmp1, secondYCord = tmp2;

        int difference = abs(firstXCord - secondXCord) + abs(firstYCord - secondYCord);
        // Error checking
        if(difference != 1 && difference != 2){
            ROS_ERROR("Astar global planner: Error in getMoveCost - difference not valid");
            return 1.0;
        }
        if(difference == 1)
            return 1.0;
        else
            return 1.4;
    }

    double RRTStarPlanner::getHeuristic(int cell_index, int goal_index)
    {
        unsigned int tmp1, tmp2;
        costmap_->indexToCells(cell_index, tmp1, tmp2);
        int startX = tmp1, startY = tmp2;
        costmap_->indexToCells(goal_index, tmp1, tmp2);
        int goalX = tmp1, goalY = tmp2;

        return hypot(goalX - startX,goalY - startY);//abs(goalY - startY) + abs(goalX - startX);
    }

    bool RRTStarPlanner::isInBounds(int x, int y)
    {
        if( x < 0 || y < 0 || x >= height || y >= width)
            return false;
        return true;
    }

    vector<int> RRTStarPlanner::get_neighbors(int current_cell)
    {
        vector<int> neighborIndexes;

        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                unsigned tmp1, tmp2;
                costmap_->indexToCells(current_cell, tmp1, tmp2);
                int nextX = tmp1 + i;
                int nextY = tmp2 + j;
                int nextIndex = costmap_->getIndex(nextX, nextY);
                if(!( i == 0 && j == 0) && isInBounds(nextX, nextY) && OGM[nextIndex])
                {
                    neighborIndexes.push_back(nextIndex);
                }
            }
        }
        return neighborIndexes;
    }
    bool operator <(const NodeA& x, const NodeA& y) {
        return x.cost < y.cost;
    }
}  // namespace rrt_star_global_planner
