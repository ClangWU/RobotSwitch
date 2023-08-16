#include <geometry_msgs/Twist.h>
#include <mutex> 
// #include <geometry_msgs/PoseStamped.h>
#include "ros/ros.h"
using namespace std;
std::mutex mtx;
static ros::Publisher* vel_puber_ptr = nullptr;
geometry_msgs::Twist combined_velocity;

void left_recv_callback(const geometry_msgs::Twist::ConstPtr& msg){
  mtx.lock();
    combined_velocity.linear.x = msg->linear.z;
    vel_puber_ptr->publish(combined_velocity);
  mtx.unlock();
}
void right_recv_callback(const geometry_msgs::Twist::ConstPtr& msg){
  mtx.lock();
    combined_velocity.linear.y = msg->linear.x;
    combined_velocity.linear.z = msg->linear.y;
    vel_puber_ptr->publish(combined_velocity);
  mtx.unlock();
}
void ahrs_recv_callback(const geometry_msgs::Twist::ConstPtr& msg){
  mtx.lock();
      combined_velocity.angular.x = msg->angular.x;
      combined_velocity.angular.y = msg->angular.y;
      combined_velocity.angular.z = msg->angular.z;
      vel_puber_ptr->publish(combined_velocity);
  mtx.unlock();
}

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "center_node"); //初始化节点
  ros::NodeHandle nh;
  ros::Publisher pub = 
  nh.advertise<geometry_msgs::Twist>("/cartesian_velocity_node_controller/cartesian_velocity", 10);
  vel_puber_ptr = &pub;

  ros::Subscriber left_suber = nh.subscribe("/left_velocity", 100, left_recv_callback);
  ros::Subscriber right_suber = nh.subscribe("/right_velocity", 100, right_recv_callback);
  ros::Subscriber ahrs_suber = nh.subscribe("/ahrs_velocity", 100, ahrs_recv_callback);
    
  ros::AsyncSpinner spinner(3); // 3 thread
  spinner.start(); 
  ros::Rate loop_rate(250); // 2 ms

  while(ros::ok())
  {
    ros::spinOnce(); 
  }
  ros::waitForShutdown();
}




