#include "teleop_mcu/teleop_mcu_control.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
using namespace std;
// bool flag = false;
void ArmPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  // Here you would handle the incoming pose message, for example:
  // Extract the pose data from the message
  geometry_msgs::PoseStamped arm_pose = *msg;

  // Implement any processing you need on the pose data here
  // ...
}
int main(int argc, char** argv) 
{
  ros::init(argc, argv, "teleop_mcu_node"); //初始化节点
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~"); 

  std::string teleop_port_;
  int teleop_baud_;
  private_nh.param("teleop_port_", teleop_port_, std::string("/dev/ttyACM0"));
  private_nh.param("teleop_baud_", teleop_baud_, 115200);
  
  ros::Publisher teleop_publisher = 
  nh.advertise<std_msgs::Int32>("/robot_teleop", 10);

  ros::Publisher grip_publisher = 
  nh.advertise<std_msgs::Int32>("/robot_grip", 10);

  ros::Publisher  robot_pose_publisher =
  nh.advertise<geometry_msgs::PoseStamped>("/robot_pose", 10);

  ros::Subscriber sub = 
  nh.subscribe("/arm_pose", 10, &ArmPoseCallback);

  TeleopControl effector;
  effector.set_port(teleop_baud_, teleop_port_);//end effector
  effector.teleop_port.open();

  ros::Rate loop_rate(200); // 5 ms
  
  while(ros::ok())
  {
    effector.teleop_data = effector.port_manager.filter(effector.teleop_port.readStruct<TeleopData>(0x44, 0x55, 12));
    std_msgs::Int32 teleop_msg;
    teleop_msg.data = effector.teleop_data._cmd;
    teleop_publisher.publish(teleop_msg); 

    loop_rate.sleep();
  }
  ros::waitForShutdown();
}