#include "teleop_mcu/teleop_mcu_control.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include "autobot.h"
#include <Eigen/Dense>
#include <math.h>
#include <iostream>
#include <chrono>
#include <thread>
static double *state = NULL;
static double *action = NULL;
void ObsCallback((const std_msgs::Float32MultiArray::ConstPtr& msg))
{
  state = msg;
}

ros::Rate loop_rate(200); // 5 ms

int main() {
    ros::init(argc, argv, "teleop_mcu_node"); //初始化节点
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~"); 
    ros::Publisher  robot_pose_publisher =
    nh.advertise<geometry_msgs::PoseStamped>("/cartesian_impedance_controller/desired_pose", 10);
    ros::Subscriber obs_sub = 
    nh.subscribe("/observation", 10, &ObsCallback);
    Autocut obj;

  while(ros::ok())
  {
    obj.update(state, action);
    robot_pose_publisher.publish
    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::waitForShutdown();
}
