// Include statements 
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_broadcaster.h>
// #include <gazebo_ros/gazebo_ros_api_plugin.h>
#include <iostream>
 
using namespace std;
 
// Initialize ROS publishers
ros::Publisher pub;

 
// // Take initialpose as input and publish initial_2d
// void handle_initial_pose(const geometry_msgs::PoseWithCovarianceStamped &pose) {
//   geometry_msgs::PoseStamped rpyPose;
//   rpyPose.header.frame_id = "map";
//   rpyPose.header.stamp = pose.header.stamp;
//   rpyPose.pose.position.x = pose.pose.pose.position.x;
//   rpyPose.pose.position.y = pose.pose.pose.position.y;
//   rpyPose.pose.position.z = 0;
//   tf::Quaternion q(0, 0, pose.pose.pose.orientation.z, pose.pose.pose.orientation.w);
//   tf::Matrix3x3 m(q);
//   double roll, pitch, yaw;
//   m.getRPY(roll, pitch, yaw);
//   rpyPose.pose.orientation.x = 0;
//   rpyPose.pose.orientation.y = 0;
//   rpyPose.pose.orientation.z = yaw;
//   rpyPose.pose.orientation.w = 0;
//   pub2.publish(rpyPose);
// }
 
int main(int argc, char **argv) {
  ros::init(argc, argv, "init_pose");
  ros::NodeHandle node;
  ros::NodeHandle node_service;
  // ros::ServiceClient client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  // gazebo_msgs::GetModelState objstate;
  geometry_msgs::Pose pose; //init pose of robot
  ros::Publisher pub = node.advertise<geometry_msgs::PoseStamped>("initialpose", 1);
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    geometry_msgs::PoseStamped rpyPose;
    // objstate.request.model_name = "turtlebot3";
    // objstate.request.relative_entity_name = "world"; 
    // if ((client.call(objstate) && objstate.response.success)){
    //   pose = objstate.request.pose;
    //   std::cout << "Got the initial pose!" << std::endl;
    // }
    rpyPose.header.frame_id = "map";
    // rpyPose.header.stamp = 0;
    rpyPose.pose.position.x = -2.8;
    rpyPose.pose.position.y = 1;
    rpyPose.pose.position.z = 0;
    rpyPose.pose.orientation.x = 0;
    rpyPose.pose.orientation.y = 0;
    rpyPose.pose.orientation.z = 0;
    rpyPose.pose.orientation.w = 1;
    pub.publish(rpyPose);
    std::cout << "Successfully sent pose" << std::endl;
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}