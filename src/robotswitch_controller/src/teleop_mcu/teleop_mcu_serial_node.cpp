#include "teleop_mcu/teleop_mcu_control.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
using namespace std;
static int start_flag = 0;
static geometry_msgs::PoseStamped delta_arm_pose;
static geometry_msgs::PoseStamped curr_arm_pose;
static geometry_msgs::PoseStamped init_arm_pose;

void ArmPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  curr_arm_pose = *msg;
}
int main(int argc, char** argv) 
{
  ros::init(argc, argv, "teleop_mcu_node"); //初始化节点
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~"); 

  std::string teleop_port_;
  int teleop_baud_;
  private_nh.param("teleop_port_", teleop_port_, std::string("/dev/ttyACM0"));
  private_nh.param("teleop_baud_", teleop_baud_, 921600);
  
  ros::Publisher teleop_publisher = 
  nh.advertise<std_msgs::Int32>("/robot_teleop", 10);

  ros::Publisher grip_publisher = 
  nh.advertise<std_msgs::Int32>("/robot_grip", 10);

  ros::Publisher  robot_pose_publisher =
  nh.advertise<geometry_msgs::PoseStamped>("/cartesian_impedance_controller/desired_pose", 10);

  ros::Subscriber sub = 
  nh.subscribe("/arm_pose", 10, &ArmPoseCallback);

  TeleopControl effector;
  effector.set_port(teleop_baud_, teleop_port_);//end effector
  effector.teleop_port.open();

  // Eigen::Quaterniond robot_hand_initial_quaternion(0.0, 1.0, 0, 0);
  // Eigen::Quaterniond q_
  ros::Rate loop_rate(200); // 5 ms
  
  while(ros::ok())
  {
    effector.teleop_data = effector.port_manager.filter(effector.teleop_port.readStruct<TeleopData>(0x44, 0x55, 12));
    std_msgs::Int32 teleop_msg;
    teleop_msg.data = effector.teleop_data._cmd;
    start_flag = teleop_msg.data;
    if (start_flag == 0){
      init_arm_pose = curr_arm_pose; 
      // robot_hand_initial_quaternion;
      // = Eigen::Quaterniond( curr_arm_pose.pose.orientation.w, curr_arm_pose.pose.orientation.x, 
      //                       curr_arm_pose.pose.orientation.y, curr_arm_pose.pose.orientation.z);
    }
    else{
      /*
            //前臂与上臂垂直
            Eigen::Quaterniond robot_fore_arm_initial_quaternion(0.7071,0,0,0.7071);
      */
      delta_arm_pose.pose.position.x = curr_arm_pose.pose.position.x - init_arm_pose.pose.position.x;
      //due to the direction
      delta_arm_pose.pose.position.y = curr_arm_pose.pose.position.y - init_arm_pose.pose.position.y;
      delta_arm_pose.pose.position.z = (curr_arm_pose.pose.position.z - init_arm_pose.pose.position.z);
      delta_arm_pose.pose.orientation = curr_arm_pose.pose.orientation;
      robot_pose_publisher.publish(delta_arm_pose);
      // ROS_INFO("\033[32mThis text will be green!\033[0m");
    }
    teleop_publisher.publish(teleop_msg); 
    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::waitForShutdown();
}