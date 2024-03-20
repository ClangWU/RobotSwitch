#include "teleop_mcu/teleop_mcu_control.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <Eigen/Dense>
#include <math.h>
#include <fstream> // 添加用于文件操作的头文件
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
  float force_y;
  float force_z;
  private_nh.param("teleop_port_", teleop_port_, std::string("/dev/ttyUSB0"));
  private_nh.param("teleop_baud_", teleop_baud_, 921600);
  
  ros::Publisher teleop_publisher = 
  nh.advertise<std_msgs::Int32>("/robot_teleop", 10);

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
  std::ofstream outfile("./force_data.txt");
  Eigen::Quaterniond q2angle;
  double _w,_x,_y,_z;
  double cy, sy, cp, sp, cr, sr;
  double roll, pitch, yaw;
  ros::Rate loop_rate(200); // 5 ms
  
  while(ros::ok())
  {
    forceband.teleop_data = forceband.port_manager.filter(forceband.teleop_port.readStruct<TeleopData>(0x44, 0x55, 20));
    std_msgs::Int32 teleop_msg;
    teleop_msg.data = forceband.teleop_data._grip;
    start_flag = forceband.teleop_data._start;

    ForceData _data;
    force_y = fext_msg.wrench.force.y;
    force_z = fext_msg.wrench.force.z;

    // Calculate the resultant force using Pythagorean theorem
    float resultant_force = sqrt(pow(force_y, 2) + pow(force_z, 2));
    // Calculate the angle with the horizontal in degrees
    float angle_with_horizontal = 180 + atan2(force_z, force_y) * (180.0 / M_PI);
    // printf("angle_with_horizontal: %f\n", angle_with_horizontal);
    // printf("resultant_force: %f\n", resultant_force);
    // Adjust angle to be in the range 0 to 359 degrees
    // if (angle_with_horizontal < 0) {
    //     angle_with_horizontal += 360.0;
    // }
    // ROS_INFO("Resultant Force: %f", resultant_force);
    // ROS_INFO("Angle with Horizontal: %f degrees", angle_with_horizontal);
    _data._force = resultant_force;       
    _data._theta = angle_with_horizontal;

    if (forceband_port_ptr != nullptr)
    {
      forceband_port_ptr->writeStruct(_data);
    }

    if (start_flag == 0x00){ //no msg publish
      init_arm_pose = curr_arm_pose; //
    }
    else if(start_flag == 0x01){
      printf("start_flag: %d\n", start_flag);
      /*
            //前臂与上臂垂直
            Eigen::Quaterniond robot_fore_arm_initial_quaternion(0.7071,0,0,0.7071);
      */
      delta_arm_pose.pose.position.x = 0;
      delta_arm_pose.pose.position.y = (curr_arm_pose.pose.position.x - init_arm_pose.pose.position.x);
      delta_arm_pose.pose.position.z = (curr_arm_pose.pose.position.z - init_arm_pose.pose.position.z);
      _w = curr_arm_pose.pose.orientation.w;
      _x = curr_arm_pose.pose.orientation.x;
      _y = curr_arm_pose.pose.orientation.y;
      _z = curr_arm_pose.pose.orientation.z;

      roll = std::atan2(2.0 * (_w * _x + _y * _z), 1.0 - 2.0 * (_x * _x + _y * _y));
      double roll_degrees = roll * 180.0 / M_PI + 180.0; 
      if (roll_degrees > 180.0 && roll_degrees < 355.0)//左偏
      {
        roll = M_PI - 0.0873; 
      }
      
      pitch = 0;
      yaw = 0;

      cy = cos(yaw * 0.5);
      sy = sin(yaw * 0.5);
      cp = cos(pitch * 0.5);
      sp = sin(pitch * 0.5);
      cr = cos(roll * 0.5);
      sr = sin(roll * 0.5);

      curr_arm_pose.pose.orientation.w = cr * cp * cy + sr * sp * sy;
      curr_arm_pose.pose.orientation.x = sr * cp * cy - cr * sp * sy;
      curr_arm_pose.pose.orientation.y = cr * sp * cy + sr * cp * sy;
      curr_arm_pose.pose.orientation.z = cr * cp * sy - sr * sp * cy;
      // euler_angle[2]
      printf("roll %f\n", roll);
      delta_arm_pose.pose.orientation = curr_arm_pose.pose.orientation;
      robot_pose_publisher.publish(delta_arm_pose);
    }    
    else {}

    teleop_publisher.publish(teleop_msg); 
    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::waitForShutdown();
}