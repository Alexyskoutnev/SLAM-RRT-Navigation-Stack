/*
  Copyright 2021 - Rafael Barreto
*/

#include "rrt_star_global_planner/collision_detector.hpp"
#include <ros/console.h> // for logging
#include <ros/ros.h> // for creating the node handle and setting up the publisher/subscriber


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
    ROS_INFO("No costmap loaded");
    return false;
  }

 

  int mx, my;
  worldToMap(wx, wy, mx, my);

  if ((mx < 0) || (my < 0) || (mx >= costmap_->getSizeInCellsX()) || (my >= costmap_->getSizeInCellsY()))
    return true;

 // int m2x, m2y;
  //worldToMap(-2.13, 2.11, m2x, m2y);
  // getCost returns unsigned char
  unsigned int cost = static_cast<int>(costmap_->getCost(mx, my));

  //unsigned int cost_test = static_cast<int>(costmap_->getCost(m2x, m2y));
 // ROS_INFO("THE COST OF PASSAGE is %u", cost_test);

  if (cost >= 254){
    ROS_INFO("Cost at (%f, %f) is %u", wx, wy, cost);

    ROS_INFO("Cost is greater than 0");
    return true;
  }

  return false;
}

bool CollisionDetector::isThereObstacleBetween(const Node &node, const std::pair<double, double> &point) {
  // In case of no costmap loaded
  if (costmap_ == nullptr) {
    // there is NO obstacles
    ROS_INFO("no costmpa loaded in isThereObstacleBetween");
    return false;
  }

  float dist = euclideanDistance2D(node.x, node.y, point.first, point.second);
  if (dist < resolution_) {
    ROS_INFO("distance is less than resolution");
    return (isThisPointCollides(point.first, point.second)) ? true : false;
  } else {
    int steps_number = static_cast<int>(floor(dist/resolution_));
    float theta = atan2(node.y - point.second, node.x - point.first);
    std::pair<float, float> p_n;
    for (int n = 1; n < steps_number; n++) {
      p_n.first = node.x + n*resolution_*cos(theta);
      p_n.second = node.y + n*resolution_*sin(theta);
      if (isThisPointCollides(p_n.first, p_n.second)){
        ROS_INFO("COLLIDING BETWEEN PATH");
        return true;
      }
        
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
