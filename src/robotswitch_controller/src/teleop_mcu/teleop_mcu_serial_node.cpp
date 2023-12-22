#include "teleop_mcu/teleop_mcu_control.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
using namespace std;
static int start_flag = 0;
static geometry_msgs::PoseStamped delta_arm_pose;
static geometry_msgs::PoseStamped curr_arm_pose;
static geometry_msgs::PoseStamped init_arm_pose;
static geometry_msgs::WrenchStamped fext_msg;
static SerialPort* forceband_port_ptr = nullptr;

void ArmPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  curr_arm_pose = *msg;
}

void FextCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  fext_msg = *msg;
}

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "teleop_mcu_node"); //初始化节点
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~"); 

  std::string teleop_port_;
  int teleop_baud_;
  private_nh.param("teleop_port_", teleop_port_, std::string("/dev/ttyUSB0"));
  private_nh.param("teleop_baud_", teleop_baud_, 921600);
  
  ros::Publisher teleop_publisher = 
  nh.advertise<std_msgs::Int32>("/robot_teleop", 10);

  ros::Publisher grip_publisher = 
  nh.advertise<std_msgs::Int32>("/robot_grip", 10);

  ros::Publisher  robot_pose_publisher =
  nh.advertise<geometry_msgs::PoseStamped>("/cartesian_impedance_controller/desired_pose", 10);
  ros::Subscriber armpose_sub = 
  nh.subscribe("/arm_pose", 10, &ArmPoseCallback);
  ros::Subscriber fext_sub = 
  nh.subscribe("/franka_state_controller/F_ext", 10, &FextCallback);

  TeleopControl forceband;
  forceband.set_port(teleop_baud_, teleop_port_);//forceband
  forceband.teleop_port.open();
  forceband_port_ptr = &forceband.teleop_port;

  ros::Rate loop_rate(10); // 5 ms
  
  while(ros::ok())
  {
    forceband.teleop_data = forceband.port_manager.filter(forceband.teleop_port.readStruct<TeleopData>(0x44, 0x55, 20));
    std_msgs::Int32 teleop_msg;
    teleop_msg.data = forceband.teleop_data._grip;
    start_flag = teleop_msg.data;

    ForceData _data;
    _data._force_y = fext_msg.wrench.force.y;
    _data._force_z = fext_msg.wrench.force.z;
    if (forceband_port_ptr != nullptr)
    {
        forceband_port_ptr->writeStruct(_data);
    }

    if (start_flag == 0){
      init_arm_pose = curr_arm_pose; 
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