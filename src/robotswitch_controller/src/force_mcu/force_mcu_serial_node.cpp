#include "force_mcu/force_mcu_control.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
using namespace std;
// bool flag = false;
int main(int argc, char** argv) 
{
  ros::init(argc, argv, "force_mcu_node"); //初始化节点
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~"); 

  std::string force_port_;
  int force_baud_;
  private_nh.param("force_port_", force_port_, std::string("/dev/ttyACM0"));
  private_nh.param("force_baud_", force_baud_, 115200);
  
  ros::Publisher force_publisher = 
  nh.advertise<std_msgs::Float32>("/end_effector_force", 10);
  ForceControl effector;
  effector.set_port(force_baud_, force_port_);//end effector
  effector.force_port.open();

  ros::Rate loop_rate(200); // 5 ms
  
  while(ros::ok())
  {
    effector.force_data = effector.port_manager.filter(effector.force_port.readStruct<ForceData>(0x44, 0x55, 12));
    std_msgs::Float32 force_msg;
    force_publisher.publish(force_msg);

    // ros::Time current_time = ros::Time::now();
    // std::stringstream ss;
    // ss << "Current time: " << current_time;
    // ROS_WARN("%s", ss.str().c_str());
    // ros::spinOnce(); 
    loop_rate.sleep();
  }
  ros::waitForShutdown();
}