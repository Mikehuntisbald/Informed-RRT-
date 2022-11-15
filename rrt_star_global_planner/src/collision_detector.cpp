/*
  Copyright 2021 - Rafael Barreto
*/

#include "collision_detector.hpp"

namespace rrt_star_global_planner {

CollisionDetector::CollisionDetector(costmap_2d::Costmap2D* costmap) : costmap_(costmap) {
  if (costmap_ != nullptr) {
    resolution_ = costmap_->getResolution();
    origin_x_ = costmap_->getOriginX();
    origin_y_ = costmap_->getOriginY();
  }
}

bool CollisionDetector::isThisPointCollides(float wx, float wy) {
  // In case of no costmap loaded
  if (costmap_ == nullptr) {
    // no collision
    return false;
  }

  int mx, my;
  worldToMap(wx, wy, mx, my);
//
//    for (int i = mx-5; i <mx+5 ; ++i) {
//        for (int j = my-5; j <my+5 ; ++j) {
//            if ((i < 0) || (j < 0) || (i >= costmap_->getSizeInCellsX()) || (j >= costmap_->getSizeInCellsY())){
//                continue;
//            }else{
//                unsigned int cost = static_cast<int>(costmap_->getCost(i, j));
//
//                if (cost > 80)
//                    return true;
//            }
//        }
//    }
  if ((mx < 0) || (my < 0) || (mx >= costmap_->getSizeInCellsX()) || (my >= costmap_->getSizeInCellsY()))
    return true;

  // getCost returns unsigned char
  unsigned int cost = static_cast<int>(costmap_->getCost(mx, my));

  if (cost > 80)
    return true;

  return false;
}

bool CollisionDetector::isThereObstacleBetween(const Node &node, const std::pair<double, double> &point) {
  // In case of no costmap loaded
  if (costmap_ == nullptr) {
    // there is NO obstacles
    return false;
  }

  float dist = euclideanDistance2D(node.x, node.y, point.first, point.second);
  if (dist < resolution_) {
    return isThisPointCollides(point.first, point.second);
  } else {
    int steps_number = static_cast<int>(ceil(dist/resolution_));
    float theta = atan2(point.second-node.y, point.first-node.x);
    std::pair<float, float> p_n;
    //std::cout<<"from"<<node.x<<","<<node.y<< "to"<<point.first<<","<<point.second<<std::endl;
    for (int n = 0; n <= steps_number; n++) {
      p_n.first = node.x + n*resolution_*cos(theta);
      p_n.second = node.y + n*resolution_*sin(theta);
      //std::cout<<p_n.first<<","<<p_n.second<< std::endl;
      if (isThisPointCollides(p_n.first, p_n.second))
        return true;
    }
    //std::cout<<p_n.first<<" and "<<p_n.second<<" compared "<<point.first<<point.second<<std::endl;
      if (isThisPointCollides(point.first, point.second)){
          return true;
      }
    return false;
  }
}

bool CollisionDetector::isThereObstacleBetween(const Node &node1, const Node &node2) {
  return isThereObstacleBetween(node1, std::make_pair(node2.x, node2.y));
}

void CollisionDetector::worldToMap(float wx, float wy, int& mx, int& my) {
  if (costmap_ != nullptr) {
    mx = (wx - origin_x_) / resolution_;
    my = (wy - origin_y_) / resolution_;
  }
}

}  // namespace rrt_star_global_planner
