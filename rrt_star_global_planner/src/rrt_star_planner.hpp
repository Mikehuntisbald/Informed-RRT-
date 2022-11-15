/*
  Copyright 2021 - Rafael Barreto
*/

#ifndef RRT_STAR_GLOBAL_PLANNER_RRT_STAR_PLANNER_HPP_  // NOLINT
#define RRT_STAR_GLOBAL_PLANNER_RRT_STAR_PLANNER_HPP_

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
//#include <astar_planner/astar_planner.h>
#include <cmath>
#include <string>
#include <vector>
#include <list>
#include <utility>
#include <memory>

#include "node.hpp"
#include "rrt_star.hpp"
#include "random_double_generator.hpp"
#define infinity 1.0e10
using namespace std;
namespace rrt_star_global_planner {
    struct NodeA{
        double cost;
        double gCost;///
        double hCost;///
        int index;
        Eigen::Isometry3d pose;
        int angle;///
        unsigned int x;///
        unsigned int y;///
        int camefrom;///
        Node* pre;///
        double v;
        double w;
        vector<geometry_msgs::PoseStamped> trajectory;///

        double getcost(){
            return gCost+hCost;
        }
    };
/**
 * @class RRTStarPlanner
 * @brief Provides a ROS rrt* global planner plugin
 */
class RRTStarPlanner : public nav_core::BaseGlobalPlanner {
 public:
  RRTStarPlanner();

  /**
   * @brief  Constructor for the RRTStarPlanner object
   * @param  name The name of this planner
   * @param  costmap A pointer to the ROS wrapper of the costmap to use
   */
  RRTStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief  Constructor for the RRTStarPlanner object
   * @param  name The name of this planner
   * @param  costmap A pointer to the costmap to use
   * @param  global_frame The global frame of the costmap
   */
  RRTStarPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);

  /**
   * @brief  Initialization function for the RRTStarPlanner object
   * @param  name The name of this planner
   * @param  costmap A pointer to the ROS wrapper of the costmap to use for planning
   */
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief  Initialization function for the RRTStarPlanner object
   * @param  name The name of this planner
   * @param  costmap A pointer to the costmap to use for planning
   * @param  global_frame The global frame of the costmap
   */
  void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);

  /**
   * @brief Given a start and goal pose in the world, compute a plan
   * @param start The start pose 
   * @param goal The goal pose 
   * @param plan The plan... filled by the planner
   * @return True if a valid plan was found, false otherwise
   */
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan);  // NOLINT

  void computeFinalPlan(std::vector<geometry_msgs::PoseStamped>& plan,  // NOLINT
                        const std::list<std::pair<float, float>> &path);
    void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
    static int factorial(int n) {
        int ret = 1;
        for (int i = 1; i <= n; ++i) {
            ret *= i;
        }
        return ret;
    }
    pair<double, double> CalcTerm(int order, geometry_msgs::PoseStamped &pose, int i, double t){
        //ROS_INFO("order = %d", order);
        double comb = (factorial(order)/(factorial(i)*factorial(order-i)));
        //ROS_INFO("Comb = %f", comb);
        //ROS_INFO("Pose x = %f,y = %f",pose.pose.position.x,pose.pose.position.y);
        pair<double, double> pair;
        pair.first = comb*pose.pose.position.x*pow(1.0-t, order-i)* pow(t,i);
        pair.second = comb*pose.pose.position.y*pow(1.0-t, order-i)* pow(t,i);


        //ROS_INFO("x = %f, y = %f",pair.first,pair.second);
        //ROS_INFO("smoothed x=%f, y=%f", pair.first, pair.second);
        return pair;
    }
    bool CalcSpline(vector<geometry_msgs::PoseStamped> &bestTraj, vector<geometry_msgs::PoseStamped> &smoothTraj);
    bool CalcSplineA(vector<geometry_msgs::PoseStamped> &bestTraj, vector<geometry_msgs::PoseStamped> &smoothTraj);
    double getHeuristic(int cell_index, int goal_index);
    vector<int> get_neighbors(int current_cell);
    double getMoveCost(int firstIndex, int secondIndex);

    bool isInBounds(int x, int y);
    bool aStar(const geometry_msgs::PoseStamped& start,
                               const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
 private:
    ros::Publisher plan_pub_;

    costmap_2d::Costmap2D* costmap_{nullptr};
  bool initialized_{false};
  int max_num_nodes_;
  int min_num_nodes_;
  double epsilon_;
  float map_width_;
  float map_height_;
  double radius_;
  double goal_tolerance_;
  double cost_;
  bool search_specific_area_{true};
  std::string global_frame_;
  std::shared_ptr<RRTStar> planner_;
    std::vector<geometry_msgs::PoseStamped>bestplan_;
//astar
    int width;
    int height;
    int map_size;
    vector<bool> OGM;
};

}  // namespace rrt_star_global_planner

#endif  // RRT_STAR_GLOBAL_PLANNER_RRT_STAR_PLANNER_HPP_  // NOLINT
