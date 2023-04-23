// Include statements 
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include "../include/navigation_interface/init_pose.h"
 
using namespace std;
 
ros::Publisher pub;
double start_x = 0.0;
double start_y = 0.0;
double start_z = 0.0;
double start_o_x = 0.0;
double start_o_y = 0.0;
double start_o_z = 0.0;
double start_o_w = 0.0;

void model_states_callback(gazebo_msgs::ModelStates model_states)
{
    auto pose = model_states.pose;
    start_x = pose[2].position.x;
    start_y = pose[2].position.y;
    start_z = pose[2].position.z;
    start_o_x = pose[2].orientation.x;
    start_o_y = pose[2].orientation.y;
    start_o_z = pose[2].orientation.z;
    start_o_w = pose[2].orientation.w;
}
 
int main(int argc, char **argv) {
  ros::init(argc, argv, "init_pose");
  ros::NodeHandle node;
  ros::NodeHandle n;
  ros::Subscriber model_states_subscriber = n.subscribe("/gazebo/model_states", 1, model_states_callback);
  ros::Publisher pub = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
  ros::Rate loop_rate(10);
  int calls = 0;
  while (ros::ok()) {
    std::string fixed_frame = "map";
    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.frame_id = fixed_frame;
    pose.header.stamp = ros::Time::now();
    pose.pose.pose.position.x = start_x;
    pose.pose.pose.position.y = start_y;
    pose.pose.pose.position.z = start_z;
    tf::Quaternion quat(start_o_x, start_o_y, start_o_z, start_o_w);
    tf::quaternionTFToMsg(quat, pose.pose.pose.orientation);
    pose.pose.covariance[6*0+0] = 0.01;
    pose.pose.covariance[6*1+1] = 0.01;
    pose.pose.covariance[6*5+5] = 0.01;
    pub.publish(pose);
    std::cout << "Successfully sent pose" << std::endl;
    ros::spinOnce();
    loop_rate.sleep();
    calls++;
    if (calls > 10)
      break;
  }
  return 0;
}