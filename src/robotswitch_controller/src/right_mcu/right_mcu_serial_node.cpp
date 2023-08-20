#include <mcu_common/common_data.h>
#include <mcu_common/serial_node.h>
#include "right_mcu/right_mcu_control.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include "std_msgs/Float32.h"
using namespace std;

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "right_mcu_node"); //初始化节点
  RightControl right;
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~"); 

  std::string right_port_;
  int right_baud_;
  private_nh.param("right_port_", right_port_, std::string("/dev/ttyUSB2"));
  private_nh.param("right_baud_", right_baud_, 115200);
  geometry_msgs::PoseStamped rs_msg;
  ros::Publisher pub = 
  nh.advertise<geometry_msgs::PoseStamped>("/right_velocity", 10); // 创建一个发布器，话题名称是"force_data"

  right.set_port(right_baud_, right_port_);
  right.port.open();
  ros::Rate loop_rate(100); // 2 ms

  while(ros::ok())
  {
    if(right.port.isOpen()){
      right.right_data = right.port_manager.filter(right.port.readStruct<RightData>(0x44, 0x55));
      rs_msg.pose.position.x = right.right_data._x; 
      rs_msg.pose.position.y = right.right_data._z; 
      pub.publish(rs_msg); 
    }
    ros::spinOnce(); 
    loop_rate.sleep();
  }
  ros::waitForShutdown();
}




