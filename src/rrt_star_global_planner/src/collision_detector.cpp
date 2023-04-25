/*
  Copyright 2021 - Rafael Barreto
*/

#include "rrt_star_global_planner/collision_detector.hpp"
#include <ros/console.h> // for logging
#include <ros/ros.h> // for creating the node handle and setting up the publisher/subscriber


namespace rrt_star_global_planner {

// gets costmap2D representation of environment
CollisionDetector::CollisionDetector(costmap_2d::Costmap2D* costmap) : costmap_(costmap) {
  if (costmap_ != nullptr) {
    resolution_ = costmap_->getResolution();
    origin_x_ = costmap_->getOriginX();
    origin_y_ = costmap_->getOriginY();
  }
}


// point in world coordinates collide with obstacle environment in costmap
bool CollisionDetector::isThisPointCollides(float wx, float wy) {
  // In case of no costmap loaded
  if (costmap_ == nullptr) {
    // no collision
    ROS_INFO("No costmap loaded");
    return false;
  }

  int mx, my;
  worldToMap(wx, wy, mx, my);

  // checks to be sure that a given point is inside the bounds of map
  // if not return collision for that point
  if ((mx < 0) || (my < 0) || (mx >= costmap_->getSizeInCellsX()) || (my >= costmap_->getSizeInCellsY()))
    return true;

  // get cost of point from costmap and check if it is within the inflated area
  unsigned int cost = costmap_->getCost(mx, my);
 
  if (cost >= 200){
    //ROS_INFO("Cost at (%f, %f) is %u", wx, wy, cost);
    return true;
  }

  // return false if pass all checks above (meaning that it is a free space)
  return false;
}

// checks if obstacle is between a node and a point in world coordinates
bool CollisionDetector::isThereObstacleBetween(const Node &node, const std::pair<double, double> &point) {
  // In case of no costmap loaded
  if (costmap_ == nullptr) {
    return false;
  }

  float dist = euclideanDistance2D(node.x, node.y, point.first, point.second);
  if (dist < resolution_) {
    return (isThisPointCollides(point.first, point.second)) ? true : false;
  } else {
    int steps_number = static_cast<int>(floor(dist/resolution_));
    float theta = atan2(node.y - point.second, node.x - point.first);
    std::pair<float, float> p_n;
    for (int n = 1; n < steps_number; n++) {
      p_n.first = node.x + n*resolution_*cos(theta);
      p_n.second = node.y + n*resolution_*sin(theta);
      if (isThisPointCollides(p_n.first, p_n.second)){
        return true;
      }
        
    }
    return false;
  }
}


// checks if the path is clear between two nodes
bool CollisionDetector::isThereObstacleBetween(const Node &node1, const Node &node2) {
  return isThereObstacleBetween(node1, std::make_pair(node2.x, node2.y));
}


// converts a point in world coordinates to costmap coordinates
void CollisionDetector::worldToMap(float wx, float wy, int& mx, int& my) {
  if (costmap_ != nullptr) {
    mx = (wx - origin_x_) / resolution_;
    my = (wy - origin_y_) / resolution_;
  }
}

}  // namespace rrt_star_global_planner