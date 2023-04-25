/*
  Copyright 2021 - Rafael Barreto
*/

#include "rrt_star_global_planner/rrt_star.hpp"


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

}

bool RRTStar::pathPlanning(std::list<std::pair<float, float>> &path) {
  goal_reached_ = false;

  // start with root node as q_init and give node_id = 0 and parent_id = -1 to signify it is root
  Node q_init(start_point_.first, start_point_.second, 0, -1);
  nodes_.push_back(q_init);
  node_count_ = 1;

  std::pair<float, float> p_rand;
  std::pair<float, float> p_new;

  Node node_nearest;

  bool found_next;
  while(nodes_.size() < max_num_nodes_) {
  //while(!goal_reached_){
    found_next = false;

    while(!found_next) {
      do {
        p_rand = sampleFree();
      } while(cd_.isThisPointCollides(p_rand.first, p_rand.second));
      
    
      node_nearest = nodes_[getNearestNodeId(p_rand)];
      p_new = steer(node_nearest.x, node_nearest.y, p_rand.first, p_rand.second);

      if (!cd_.isThereObstacleBetween(node_nearest, p_new)) {
        Node q_new(p_new.first, p_new.second, node_count_, node_nearest.node_id);
        nodes_.push_back(q_new);
        found_next = true;
        node_count_++;
        if (euclideanDistance2D(q_new.x, q_new.y, goal_point_.first, goal_point_.second) < goal_tolerance_) {

          goal_reached_ = true;
          goal_node_ = nodes_.back();
        }
      }
    }


    if (goal_reached_) {
      ROS_INFO("GOAL HAS BEEN FOUND");
      computeFinalPath(path);
      //goal_reached_ = false; // go back to original so new goal can be made
      return true;
    }
  }
  return false;
}

  



std::pair<float, float> RRTStar::sampleFree() {
  std::pair<float, float> random_point;

  // generates a random number between 0 and 1
  double prob = static_cast<double>(rand()) / RAND_MAX;

  std::mt19937 gen(rd_());

  // goal bias of 0.10 to set to goal position
  if (prob <= 0.10) {
    random_point.first = goal_point_.first;
   random_point.second = goal_point_.second;
  } else {
    // creates a random point within the map boundaries using random generator
    random_point.first = std::uniform_real_distribution<float> (-map_width_, std::nextafter(map_width_, DBL_MAX))(gen);
    random_point.second = std::uniform_real_distribution<float> (-map_width_, std::nextafter(map_width_, DBL_MAX))(gen);
  }

  return random_point;
}

// find the id of the nearest node
int RRTStar::getNearestNodeId(const std::pair<float, float> &point){
  //ROS_INFO("getting nearest node ID");
  // safety check to make sure nodes is not empty
  if (nodes_.empty()){
    ROS_INFO("ERROR: nodes is empty when searching for nearest neighbor");
    return -1;
  }

  float dist_nearest, dist;
  Node node_nearest = nodes_[0];
  for (int i = 1; i < nodes_.size(); ++i) {
    dist_nearest = euclideanDistance2D(node_nearest.x, node_nearest.y, point.first, point.second);
    dist = euclideanDistance2D(nodes_[i].x, nodes_[i].y, point.first, point.second);
    if (dist < dist_nearest) node_nearest = nodes_[i];
  }

  return node_nearest.node_id;
}

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

void RRTStar::computeFinalPath(std::list<std::pair<float, float>> &path){
  path.clear();

  // Compute the path from the goal to the start
  Node current_node = goal_node_;

  // Final Path
  std::pair<float, float> point;

  do {
    point.first = current_node.x;
    point.second = current_node.y;
    path.push_front(point);


    current_node = nodes_[current_node.parent_id];

  } while (current_node.parent_id != -1);

  
  ROS_INFO("Final path has been computed.");
}

std::vector<Node> RRTStar::getNodes() const {
  return nodes_;
}



bool RRTStar::isGoalReached(const std::pair<float, float> &p_new) {
  return (euclideanDistance2D(p_new.first,
                              p_new.second,
                              goal_point_.first,
                              goal_point_.second) < goal_tolerance_) ? true : false;
}

}  // namespace rrt_star_global_planner
