#include <mutex> 
#include <geometry_msgs/PoseStamped.h>
#include "ros/ros.h"
using namespace std;
std::mutex mtx;
static ros::Publisher* pose_puber_ptr = nullptr;
geometry_msgs::PoseStamped combined_pose;

void left_recv_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  mtx.lock();
    combined_pose.pose.position.x = msg->pose.position.z;
    pose_puber_ptr->publish(combined_pose);
  mtx.unlock();
}
void right_recv_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  mtx.lock();
    combined_pose.pose.position.y = msg->pose.position.x;
    combined_pose.pose.position.z = msg->pose.position.y;
    pose_puber_ptr->publish(combined_pose);
  mtx.unlock();
}
void ahrs_recv_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  mtx.lock();
    combined_pose.pose.orientation.w = msg->pose.orientation.w;
    combined_pose.pose.orientation.x = msg->pose.orientation.x;
    combined_pose.pose.orientation.y = msg->pose.orientation.y;
    combined_pose.pose.orientation.z = msg->pose.orientation.z;
      pose_puber_ptr->publish(combined_pose);
  mtx.unlock();
}

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "center_node"); //初始化节点
  ros::NodeHandle nh;
  ros::Publisher pub = 
  nh.advertise<geometry_msgs::PoseStamped>("/cartesian_impedance_controller/desired_pose", 10);
  pose_puber_ptr = &pub;

  ros::Subscriber left_suber = nh.subscribe("/left_velocity", 100, left_recv_callback);
  ros::Subscriber right_suber = nh.subscribe("/right_velocity", 100, right_recv_callback);
  ros::Subscriber ahrs_suber = nh.subscribe("/qtn_pose", 100, ahrs_recv_callback);
    
  ros::AsyncSpinner spinner(3); // 3 thread
  spinner.start(); 
  ros::Rate loop_rate(250); // 2 ms

  while(ros::ok())
  {
    ros::spinOnce(); 
  }
  ros::waitForShutdown();
}




