#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <gazebo_msgs/ModelStates.h>
#include <iostream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

double start_x = 0.0;
double start_y = -0.2;
double start_z = 0.0;
double start_o_x = 0.0;
double start_o_y = 0.0;
double start_o_z = 0.0;
double start_o_w = 0.0;
bool init_flag = false;

void model_states_callback(gazebo_msgs::ModelStates model_states)
{
    if (init_flag == false){
        auto pose = model_states.pose;
        start_x = pose[2].position.x;
        start_y = pose[2].position.y;
        start_z = pose[2].position.z;
        start_o_x = pose[2].orientation.x;
        start_o_y = pose[2].orientation.y;
        start_o_z = pose[2].orientation.z;
        start_o_w = pose[2].orientation.w;
        init_flag = true;
    } 
    return;   
}

int main(int argc, char** argv){
  ros::init(argc, argv, "navigation_interface");
  MoveBaseClient ac("move_base", true);
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  bool run = true;
  bool valid_flag = true;
  int usr_choice;
  move_base_msgs::MoveBaseGoal goal;
  move_base_msgs::MoveBaseGoal goal_back;
  ros::NodeHandle n;
  ros::Subscriber model_states_subscriber = n.subscribe("/gazebo/model_states", 1, model_states_callback);
  while(run){
    
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal_back.target_pose.header.frame_id = "map";
    goal_back.target_pose.header.stamp = ros::Time::now();
    std::cout << "Where should the robot go?" << std::endl;
    std::cout << "1 = room 1 \n2 = room 2\n3 = room 3 \n4 = room 4 \n5 = room 5 \n6 = room 6 \n " << std::endl;
    std::cin >> usr_choice;
    ros::spinOnce();
    switch(usr_choice){
        case 1:
            std::cout << "Going to room 1!" << std::endl;
            goal.target_pose.pose.position.x = 5.5;
            goal.target_pose.pose.position.y = 2.0;
            goal.target_pose.pose.orientation.w = 1.0;
            valid_flag = true;
            break;
        case 2:
            std::cout << "Going to room 2!" << std::endl;
            goal.target_pose.pose.position.x = 5.8;
            goal.target_pose.pose.position.y = -2.10;
            goal.target_pose.pose.orientation.w = 1.0;
            valid_flag = true;
            break;
        case 3:
            std::cout << "Going to room 3!" << std::endl;
            goal.target_pose.pose.position.x = 1.0;
            goal.target_pose.pose.position.y = 3.0;
            goal.target_pose.pose.orientation.w = 1.0;
            valid_flag = true;
            break;
        case 4:
            std::cout << "Going to room 4!" << std::endl;
            goal.target_pose.pose.position.x = -3.0;
            goal.target_pose.pose.position.y = 1.75;
            goal.target_pose.pose.orientation.w = 1.0;
            valid_flag = true;
            break;
        case 5:
            std::cout << "Going to room 5!" << std::endl;
            goal.target_pose.pose.position.x = -6.5;
            goal.target_pose.pose.position.y = 3.0;
            goal.target_pose.pose.orientation.w = 1.0;
            valid_flag = true;
            break;
        case 6:
            std::cout << "Going to room 6!" << std::endl;
            goal.target_pose.pose.position.x = -6.5;
            goal.target_pose.pose.position.y = -1.0;
            goal.target_pose.pose.orientation.w = 1.0;
            valid_flag = true;
            break;
        default:
            std::cout << "enter a valid number btw 1-6" << std::endl;
            valid_flag = false;
            ROS_INFO_STREAM("valid flag: " << valid_flag);
        }
        ROS_INFO("Sending goal !!!");
        ac.sendGoal(goal);
        ac.waitForResult();
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO("We reached to the goal! Time to go back");
            goal.target_pose.pose.position.x = start_x;
            goal.target_pose.pose.position.y = start_y;
            goal.target_pose.pose.orientation.w = start_o_w;
            ac.sendGoal(goal);
            ac.waitForResult();
            std::cout << "Do you want to move to another spot? (Y) or (N)" << std::endl;
            char c;
            std::cin >> c;
            if (c == 'Y' || c == 'y'){
                continue;
            } else {
                run = false;
                std::cout << "Exiting" << std::endl;
            }
        }
    }
    return 0;
}