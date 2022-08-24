/*
  Copyright 2021 - Rafael Barreto
*/

#include "rrt_star.hpp"


namespace rrt_star_global_planner {

RRTStar::RRTStar(const std::pair<float, float> &start_point,
                 const std::pair<float, float> &goal_point,
                 costmap_2d::Costmap2D* costmap,
                 double goal_tolerance,
                 double radius,
                 double epsilon,
                 unsigned int max_num_nodes,
                 unsigned int min_num_nodes,
                 float map_width,
                 float map_height) : start_point_(start_point),
                                     goal_point_(goal_point),
                                     costmap_(costmap),
                                     goal_tolerance_(goal_tolerance),
                                     radius_(radius),
                                     epsilon_(epsilon),
                                     max_num_nodes_(max_num_nodes),
                                     min_num_nodes_(min_num_nodes),
                                     map_width_(map_width),
                                     map_height_(map_height),
                                     cd_(costmap) {
    nodes_.reserve(max_num_nodes_);
    ///Get the angle difference from start to goal and form a rotation matrix
    double bian = hypot((goal_point_.first-start_point_.first),(goal_point_.second-start_point_.second));
    double goal_th;
    if(goal_point_.second-start_point_.second<0){
        goal_th = -acos((goal_point_.first-start_point_.first)/bian);
    }else
        goal_th = acos((goal_point_.first-start_point_.first)/bian);
    Eigen::AngleAxisd rotation_vector(goal_th, Eigen::Vector3d(0, 0, 1));
    rotation_matrix_ = rotation_vector.toRotationMatrix();
  // Set range
  random_double_.setRange(-1, 1);
    //ROS_INFO("mapwidth is %f",map_width);
    //ROS_INFO("origin X = %f, starting point = %f", costmap_->getOriginX(), start_point.first);
}

bool RRTStar::pathPlanning(std::list<std::pair<float, float>> &path) {
  goal_reached_ = false;

  if (cd_.isThisPointCollides(goal_point_.first, goal_point_.second)) {
    ROS_ERROR("Goal point chosen is NOT in the FREE SPACE! Choose other goal!");
    return false;
  }

  // Start Node
  createNewNode(start_point_.first, start_point_.second, -1);

  std::pair<float, float> p_rand;
  std::pair<float, float> p_new;

  Node node_nearest;

    bool found_next;
    while (nodes_.size() < max_num_nodes_) {
        found_next = false;
        while (!found_next) {
            if(!goal_reached_){
                p_rand = sampleFree();  // random point in the free space
            }else p_rand = sampleCircle(c);
            node_nearest = nodes_[getNearestNodeId(p_rand)];  // nearest node of the random point
            p_new = steer(node_nearest.x, node_nearest.y, p_rand.first, p_rand.second);  // new point and node candidate
            if (!cd_.isThereObstacleBetween(node_nearest, p_new)) {
                found_next = true;
                createNewNode(p_new.first, p_new.second, node_nearest.node_id);
            }
        }

        if (!goal_reached_) {
            if (isGoalReached(p_new)) {
                goal_reached_ = true;
                //goal_node_ = nodes_.back();
                goal_node_.x= goal_point_.first;
                goal_node_.y = goal_point_.second;
                goal_node_.node_id = node_count_;
                goal_node_.parent_id = nodes_.back().node_id;
                node_count_++;
                goal_node_.cost = nodes_.back().cost+ euclideanDistance2D(nodes_.back().x,nodes_.back().y,goal_node_.x,goal_node_.y);
                c = goal_node_.cost;
                nodes_.emplace_back(goal_node_);
                Node current_node = goal_node_;
                std::pair<float, float> point;
                while (current_node.parent_id != -1){
                    point.first = current_node.x;
                    point.second = current_node.y;
                    prepath.insert(std::pair<int, Node>(current_node.node_id, current_node));

                    // update the current node
                    current_node = nodes_[current_node.parent_id];
                }
                /*goal_node_.x = goal_point_.first;
                goal_node_.y = goal_point_.second;
                goal_node_.node_id = nodes_.back().node_id;
                goal_node_.parent_id = nodes_.back().parent_id;
                goal_node_.cost = nodes_.back().cost;*/
            }
        }

    if (goal_reached_ && nodes_.size() > min_num_nodes_) {
      computeFinalPath(path);
      return true;
    }
  }
  return false;
}

std::pair<float, float> RRTStar::sampleFree() {
  std::pair<float, float> random_point;
  random_point.first = map_width_*random_double_.generate();
  random_point.second = map_height_*random_double_.generate();

  return random_point;
}

    std::pair<float, float> RRTStar::sampleCircle(double cost) {
        //ROS_INFO("Sampling Circle!!!!!!!!");
        std::pair<float, float> random_point;
        double r2;
        double theta;
        r2 = map_width_* fabs(random_double_.generate());
        theta = 2*M_PI* fabs(random_double_.generate());
        random_point.first = sqrt(r2) * cos(theta)*(cost/2);
        random_point.second = sqrt(r2) * sin(theta)*sqrt((cost/2)*(cost/2)- (euclideanDistance2D(start_point_.first,start_point_.second,goal_point_.first,goal_point_.second)/2)*(euclideanDistance2D(start_point_.first,start_point_.second,goal_point_.first,goal_point_.second)/2));
        ROS_INFO("x= %f,y= %f",random_point.first, random_point.second);

        Eigen::Vector3d vector(random_point.first,random_point.second,0);
        vector = rotation_matrix_ * vector;
        random_point.first = vector[0]+goal_point_.first+(goal_point_.first-start_point_.first)/2;
        random_point.second = vector[1]+goal_point_.second+(goal_point_.second-start_point_.second)/2;
        return random_point;
    }

int RRTStar::getNearestNodeId(const std::pair<float, float> &point) {
  float dist_nearest, dist;
  Node node_nearest = nodes_[0];
  for (int i = 1; i < nodes_.size(); ++i) {
    dist_nearest = euclideanDistance2D(node_nearest.x, node_nearest.y, point.first, point.second);
    dist = euclideanDistance2D(nodes_[i].x, nodes_[i].y, point.first, point.second);
    if (dist < dist_nearest) node_nearest = nodes_[i];
  }

  return node_nearest.node_id;
}

void RRTStar::createNewNode(float x, float y, int node_nearest_id) {
  Node new_node(x, y, node_count_, node_nearest_id);
  nodes_.emplace_back(new_node);

  if (node_nearest_id != -1) {
    // Optimize
    chooseParent(node_nearest_id);
    rewire();
  }

  node_count_++;
}

    void RRTStar::chooseParent(int node_nearest_id) {
        float cost_new_node;
        float cost_other_parent;
        float nodes_dist;

        Node parent_node = nodes_[node_nearest_id];

        Node &new_node = nodes_.back();

        for (const auto &node : nodes_) {
            if (node.node_id == new_node.node_id) continue;
            // distance between node and new_node
            nodes_dist = euclideanDistance2D(node.x, node.y, new_node.x, new_node.y);

            if (nodes_dist < radius_) {
                // current cost of new_node
                cost_new_node = parent_node.cost + euclideanDistance2D(parent_node.x, parent_node.y, new_node.x, new_node.y);

                // cost if the parent is node
                cost_other_parent = node.cost + nodes_dist;

                if (cost_other_parent < cost_new_node) {
                    if (!cd_.isThereObstacleBetween(node, new_node)) {
                        parent_node = node;
                    }
                }
            }
        }///rank potential parent nodes in radius, continue as long as find a node total cost< current parent total cost

        // Update new_node cost and its new parent
        new_node.cost = parent_node.cost + euclideanDistance2D(parent_node.x, parent_node.y, new_node.x, new_node.y);
        new_node.parent_id = parent_node.node_id;
    }


    void RRTStar::rewire() {
        float nodes_dist;
        float cost_node;

        Node new_node = nodes_.back();

        for (auto &node : nodes_) {
            // distance between node and new_node
            nodes_dist = euclideanDistance2D(node.x, node.y, new_node.x, new_node.y);
            // check if node is already the parent and if node is near for optimization
            if (node != nodes_[new_node.parent_id] && nodes_dist < radius_) {
                // cost if the parent of node is new_node
                cost_node = new_node.cost + nodes_dist;

                if (cost_node < node.cost && !cd_.isThereObstacleBetween(node, new_node)) {
                    it = prepath.find(node.node_id);
                    // update the new parent of node and its new cost
                    node.parent_id = new_node.node_id;
                    float dif = node.cost-cost_node;
                    node.cost = cost_node;
                    if (it != prepath.end()){
                        c = c - dif;
                    }
                }
            }
        }
    }

// TODO(Rafael) improve parameters name
std::pair<float, float> RRTStar::steer(float x1, float y1, float x2, float y2) {
  std::pair<float, float> p_new;
  float dist = euclideanDistance2D(x1, y1, x2, y2);
  if (dist < epsilon_) {
    p_new.first = x1;
    p_new.second = y1;
    return p_new;
  } else {
    float theta = atan2(y2 - y1, x2 - x1);
    p_new.first = x1 + epsilon_*cos(theta);
    p_new.second = y1 + epsilon_*sin(theta);
    return p_new;
  }
}

std::vector<Node> RRTStar::getNodes() const {
  return nodes_;
}

void RRTStar::computeFinalPath(std::list<std::pair<float, float>> &path) {
  path.clear();

  // Compute the path from the goal to the start
  Node current_node = goal_node_;

  // Final Path
  std::pair<float, float> point;

  do {
    point.first = current_node.x;
    point.second = current_node.y;
    path.push_front(point);

    // update the current node
    current_node = nodes_[current_node.parent_id];
  } while (current_node.parent_id != -1);
}

bool RRTStar::isGoalReached(const std::pair<float, float> &p_new) {
  return (euclideanDistance2D(p_new.first,
                              p_new.second,
                              goal_point_.first,
                              goal_point_.second) < goal_tolerance_) ? true : false;
}

}  // namespace rrt_star_global_planner
