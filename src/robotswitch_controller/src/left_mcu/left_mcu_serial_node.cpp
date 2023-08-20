#include <mcu_common/common_data.h>
#include <mcu_common/serial_node.h>
#include "left_mcu/left_mcu_control.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include <std_msgs/Float32.h>
using namespace std;
// bool flag = false;
static SerialPort* left_port_ptr = nullptr;
void left_callback(const std_msgs::Float32::ConstPtr& msg1) {
  ForceData _data;
  _data._force = msg1->data;
  left_port_ptr->writeStruct(_data);
}

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "left_mcu_node"); //初始化节点
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~"); 

  std::string left_port_;
  int left_baud_;
  private_nh.param("left_port_", left_port_, std::string("/dev/ttyUSB1"));
  private_nh.param("left_baud_", left_baud_, 115200);

  geometry_msgs::PoseStamped left_msg;
  ros::Publisher  left_pub = 
  nh.advertise<geometry_msgs::PoseStamped>("/left_velocity", 10); // 创建一个发布器，话题名称是"force_data"

  ros::Subscriber sub = nh.subscribe("/end_effector_force", 10, left_callback);

  LeftControl left;
  left.set_port(left_baud_, left_port_);
  left.left_port.open();
  left_port_ptr = &left.left_port;
  // left.set_right_port(115200, "/dev/ttyUSB2");
  // left.right_port.open();

  ros::Rate loop_rate(500); // 5 ms
  
  while(ros::ok())
  {
    left.left_data = left.port_manager.filter(left.left_port.readStruct<LeftData>(0x44, 0x55));
    left_msg.pose.position.z = left.left_data._y;
    left_pub.publish(left_msg);

    ros::spinOnce(); 
    loop_rate.sleep();
  }
  ros::waitForShutdown();
}




