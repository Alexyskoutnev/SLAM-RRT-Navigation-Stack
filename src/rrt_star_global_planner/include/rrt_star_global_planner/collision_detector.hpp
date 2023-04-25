/*
  Copyright 2021 - Rafael Barreto
*/

#ifndef RRT_STAR_GLOBAL_PLANNER_COLLISION_DETECTOR_HPP_
#define RRT_STAR_GLOBAL_PLANNER_COLLISION_DETECTOR_HPP_

#include <costmap_2d/costmap_2d.h>

#include <utility>

#include "rrt_star_global_planner/node.hpp"
#include <nav_msgs/OccupancyGrid.h>


namespace rrt_star_global_planner {

class CollisionDetector {
 public:
  explicit CollisionDetector(costmap_2d::Costmap2D* costmap);

  bool isThisPointCollides(float wx, float wy);

  // line between node and point
  bool isThereObstacleBetween(const Node &node, const std::pair<double, double> &point);

  // line between two nodes
  bool isThereObstacleBetween(const Node &node1, const Node &node2);

 private:
  void worldToMap(float wx, float wy, int& mx, int& my);

  costmap_2d::Costmap2D* costmap_{nullptr};
  double resolution_{0.01};
  double origin_x_{0.0};
  double origin_y_{0.0};
  double inflation_radius_{0.5};
};

}  // namespace rrt_star_global_planner

#endif  // RRT_STAR_GLOBAL_PLANNER_COLLISION_DETECTOR_HPP_
