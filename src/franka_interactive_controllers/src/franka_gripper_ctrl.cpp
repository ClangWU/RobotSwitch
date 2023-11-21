#include <iostream>
#include <sstream>
#include <string>

#include <franka/exception.h>

#include "ros/ros.h"
#include <std_msgs/Int32.h>
#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/franka_gripper.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>
int human_tele = 0;

void TeleCallback(const std_msgs::Int32::ConstPtr& msg) {
    // ROS_INFO("Received Tele message: %f", msg->data);
    human_tele = msg->data;
  }

int main(int argc, char** argv) {

  static bool in_open_state = false;
  static bool in_close_state = false;

  ros::init(argc, argv, "franka_gripper_run_node");
  ros::NodeHandle nh;
  ros::NodeHandle _nh("~");

  actionlib::SimpleActionClient<franka_gripper::MoveAction> ac_move("/franka_gripper/move", true);
  actionlib::SimpleActionClient<franka_gripper::GraspAction> ac_grasp("/franka_gripper/grasp", true);
  ros::Subscriber sub = nh.subscribe("/robot_teleop", 1, TeleCallback);
  ROS_INFO("Waiting for MoveAction server to start.");
  ROS_INFO("Waiting for GraspAction server to start.");
  ac_move.waitForServer();
  ac_grasp.waitForServer();
  franka_gripper::MoveGoal move_goal;
  franka_gripper::GraspGoal grasp_goal;
  ros::Rate rate(200); 

while (ros::ok())
{
    /* code */
    if (human_tele == 1 && in_open_state ==false)
    {
      std::cout << "Opening gripper" << std::endl;
      // Creating goal for moving action
      move_goal.width = 0.08;
      move_goal.speed = 0.5;
      ac_move.sendGoal(move_goal);
      in_open_state = true;
      in_close_state = false;
    }
    else if(human_tele == 3 && in_close_state ==false)
    {
        // close gripper
        std::cout << "Closing gripper" << std::endl;
        
        // Creating goal for grasping action with 50N force
        grasp_goal.width = 0.0;
        grasp_goal.speed = 0.5;
        grasp_goal.force = 50;
        grasp_goal.epsilon.inner = 0.2;
        grasp_goal.epsilon.outer = 0.2;
        ac_grasp.sendGoal(grasp_goal);
        in_close_state = true;
        in_open_state = false;
        //wait for the action to return
        ac_grasp.waitForResult(ros::Duration(3.0));
    }
      ros::spinOnce();
      rate.sleep();
  }
  // Stop the node's resources
  ros::waitForShutdown();
  // Exit tranquilly
  return 0;

}
