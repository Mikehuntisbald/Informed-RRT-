/*
  Copyright 2021 - Rafael Barreto
*/

#ifndef RRT_STAR_GLOBAL_PLANNER_RRT_STAR_HPP_  // NOLINT
#define RRT_STAR_GLOBAL_PLANNER_RRT_STAR_HPP_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <string>
#include <vector>
#include <list>
#include <utility>

#include "random_double_generator.hpp"
#include "node.hpp"
#include "collision_detector.hpp"


namespace rrt_star_global_planner {

class RRTStar {
 public:
  RRTStar(const std::pair<float, float> &start_point,
          const std::pair<float, float> &goal_point,
          costmap_2d::Costmap2D* costmap,
          double goal_tolerance,
          double radius,
          double epsilon,
          unsigned int max_num_nodes,
          unsigned int min_num_nodes,
          float map_width,
          float map_height);

  /**
   * @brief compute the RRT* path planning
   * @param path list of planar positions (x, y)
   * @return true if a path is found, false otherwise
   */
  bool pathPlanning(std::list<std::pair<float, float>> &path, double &cost);  // NOLINT

  /**
   * @brief compute random points
   * @return random planar position (x, y)
   */
  std::pair<float, float> sampleFree();

  std::pair<float, float> sampleCircle(double cost);

    /**
   * @brief Get the Index of the nearest node around the new random point
   * @param point random pointed sampled
   * @return the nearest node Index
   * @note exposed for testing purposes
   */
  int getNearestNodeId(const std::pair<float, float> &point);

  /**
   * @brief Create a new node
   * @param x cartesian coordinate of the new node
   * @param y cartesian coordinate of the new node
   * @param node_nearest_id
   * @note exposed for testing purposes
   */
  void createNewNode(float x, float y, int node_nearest_id);

  /**
   * @brief RRT* near neighbour search step. Selection of the parent node with lowest cost
   * @param node_nearest_id index of the nearest node around the new node
   * @note exposed for testing purposes
   */
  void chooseParent(int node_nearest_id);

  /**
   * @brief RRT* rewiring step. Search the best parent inside of the circular area of the new node
   * @note exposed for testing purposes
   */
  void rewire();

  // TODO(Rafael) improve method name and parameters
  // std::pair<float, float> connect(const std::pair<float, float> &p_new, int id_nearest_node);
  /** 
   * @brief Connect new point from a parent
   * @param 
   * @param 
   * @return
   * @note exposed for testing purposes
   */
  std::pair<float, float> steer(float x1, float y1, float x2, float y2);

  std::vector<Node> getNodes() const;

  void computeFinalPath(std::list<std::pair<float, float>> &path);  // NOLINT

  bool isGoalReached(const std::pair<float, float> &p_new);

 private:
    std::pair<float, float> start_point_;
    std::pair<float, float> goal_point_;
    costmap_2d::Costmap2D* costmap_{nullptr};

    std::vector<Node> nodes_;
    RandomDoubleGenerator random_double_;
    //std::uniform_real_distribution<> dist_unit_ = std::uniform_real_distribution<>(0.0, 1.0);
    //RandomDoubleGenerator random_double_x;
    //RandomDoubleGenerator random_double_y;
    int node_count_{0};
    float map_width_;
    float map_height_;
    double radius_;
    unsigned int max_num_nodes_;
    unsigned int min_num_nodes_;
    double goal_tolerance_;
    double epsilon_;
    Eigen::Matrix3d rotation_matrix_;
    bool goal_reached_{false};
    std::map<int, Node> prepath;
    std::map<int, Node> tmppath;
    std::map<int, Node>::iterator it;
    float c;
    Node goal_node_;

    CollisionDetector cd_;
};

}  // namespace rrt_star_global_planner

#endif  // RRT_STAR_GLOBAL_PLANNER_RRT_STAR_HPP_  NOLINT
