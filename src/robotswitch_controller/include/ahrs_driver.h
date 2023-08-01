#ifndef BASE_DRIVER_H_
#define BASE_DRIVER_H_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <serial/serial.h> //ROS的串口包 http://wjwwood.io/serial/doc/1.1.0/index.html

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <math.h>
#include <fstream>
#include <fdilink_data_struct.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Pose2D.h>
#include <boost/thread.hpp>
#include <string>
#include <ros/package.h>
#include <crc_table.h>

using namespace std;
namespace RobotSwitch
{
#define FRAME_HEAD 0xfc
#define FRAME_END 0xfd
#define TYPE_IMU 0x40
#define TYPE_AHRS 0x41
#define TYPE_INSGPS 0x42
#define TYPE_GEODETIC_POS 0x5c
#define TYPE_GROUND 0xf0

#define IMU_LEN  0x38   //56
#define AHRS_LEN 0x30   //48
#define INSGPS_LEN 0x48 //80
#define GEODETIC_POS_LEN 0x20 //32
#define PI 3.141592653589793
#define DEG_TO_RAD 0.017453292519943295

struct ForceData
{
    float _force;    //HX711 data
} __attribute__((packed));

struct MoveData
{
    int x_;    //move data
    int z_;
} __attribute__((packed));

struct InteractData
{
    int y_;    //interact data
} __attribute__((packed));

class RobotSwitchBringup
{
public:
  RobotSwitchBringup();
  ~RobotSwitchBringup();
  void processLoop();
  // all task process
  void ahrs_process();
  void move_process();
  void interact_process();
  void force_process();

  bool ahrs_checkCS8(int len);
  bool ahrs_checkCS16(int len);
  void ahrs_checkSN(int type);
  void ahrs_magCalculateYaw(double roll, double pitch, double &magyaw, double magx, double magy, double magz);
  void serial_init(serial::Serial *serial_, std::string _port_, int _baud_, int _timeout_);
  ros::NodeHandle nh_;

private:
  bool if_debug_;
  //sum info
  int sn_lost_ = 0;
  int crc_error_ = 0;
  uint8_t read_sn_ = 0;
  bool frist_sn_;
  int device_type_ = 1;

  //4 serial
  //imu
  serial::Serial ahrs_serial_; //声明串口对象
  std::string ahrs_serial_port_;
  int ahrs_serial_baud_;
  int ahrs_serial_timeout_;

  //interact dof
  serial::Serial interact_dof_serial_; //声明串口对象
  std::string interact_dof_serial_port_;
  int interact_dof_serial_baud_;
  int interact_dof_serial_timeout_;

  //move dof
  serial::Serial move_dof_serial_; //声明串口对象
  std::string move_dof_serial_port_;
  int move_dof_serial_baud_;
  int move_dof_serial_timeout_;

  //force dof
  serial::Serial force_dof_serial_; //声明串口对象
  std::string force_dof_serial_port_;
  int force_dof_serial_baud_;
  int force_dof_serial_timeout_;

  //data
  FDILink::imu_frame_read  imu_frame_;
  FDILink::ahrs_frame_read ahrs_frame_;
  FDILink::insgps_frame_read insgps_frame_;
  //FDILink::lanlon_frame_read latlon_frame_;
  FDILink::Geodetic_Position_frame_read Geodetic_Position_frame_;
  //frame name
  string imu_frame_id_;
  string insgps_frame_id_;
  string latlon_frame_id_;
  //topic
  string imu_topic_, mag_pose_2d_topic_;
  string latlon_topic_;
  string Euler_angles_topic_,Magnetic_topic_;
  string gps_topic_,twist_topic_,NED_odom_topic_;

  //Publisher
  ros::Publisher imu_pub_;
  ros::Publisher gps_pub_;
  ros::Publisher mag_pose_pub_;
  ros::Publisher Euler_angles_pub_;
  ros::Publisher Magnetic_pub_;
  ros::Publisher twist_pub_;
  ros::Publisher NED_odom_pub_;

  ros::Publisher imu_velocity_command_publisher;

  ForceData     _force_handle;
  MoveData        _move_handle;
  InteractData _interact_handle;

  template <typename T>
    std::vector<T> readStruct(serial::Serial *serial_, unsigned char head, unsigned char tail)
    {
    std::vector<T> vec_t;
    const int LENGTH = 24;
    const int SIZE = sizeof(T);
    unsigned char read_buffer[LENGTH] = {0};
    size_t len_result = serial_->read(read_buffer, LENGTH);
    for (size_t i = 0; (i + SIZE + 1) < len_result; i++)
    {
        if (read_buffer[i] == head && read_buffer[i + SIZE + 1] == tail)
        { 
        vec_t.push_back(*(reinterpret_cast<T *>(&read_buffer[i + 1])));
        }
    }
    return vec_t;
    }

    template <typename T>
    bool writeStruct(serial::Serial *serial_, T data_struct)
    {
        size_t len_result = serial_->write(reinterpret_cast<const uint8_t*>(&data_struct), sizeof(data_struct));
        return (sizeof(data_struct) == len_result);
    }

  template <typename T>
    T filter(const std::vector<T> &dataScope){
    if (!dataScope.empty())
    {
        return dataScope.back();
    }else{
        ROS_ERROR_STREAM("Unable to read data ");
    }
  }

  }; //RobotSwitchBringup
} // namespace RobotSwitch



#endif
