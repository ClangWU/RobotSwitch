#include "imu_mcu/imu_mcu_control.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>

using namespace std;
// bool flag = false;
int main(int argc, char** argv) 
{
  ros::init(argc, argv, "IMU_mcu_node"); //初始化节点
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~"); 

  std::string IMU_port_;
  int IMU_baud_;
  private_nh.param("IMU_port_", IMU_port_, std::string("/dev/ttyACM0"));
  private_nh.param("IMU_baud_", IMU_baud_, 115200);
  
  ros::Publisher IMU_publisher = 
  nh.advertise<sensor_msgs::Imu>("/hand_IMU", 10);
  IMUControl effector;
  effector.set_port(IMU_baud_, IMU_port_);//end effector
  effector.IMU_port.open();

  // ros::Rate loop_rate(200); // 5 ms
  
  while(ros::ok())
  {
    effector.IMU_data = effector.port_manager.filter(effector.IMU_port.readStruct<IMUData>(0x44, 0x55, 36));
    sensor_msgs::Imu IMU_msg;
    IMU_msg.orientation.w = effector.IMU_data.qw;
    IMU_msg.orientation.x = effector.IMU_data.qx;
    IMU_msg.orientation.y = effector.IMU_data.qy;
    IMU_msg.orientation.z = effector.IMU_data.qz;

    IMU_publisher.publish(IMU_msg);

    // ros::Time current_time = ros::Time::now();
    // std::stringstream ss;
    // ss << "Current time: " << current_time;
    // ROS_WARN("%s", ss.str().c_str());
    // ros::spinOnce(); 
    // loop_rate.sleep();
  }
  ros::waitForShutdown();
}