#pragma once
#include <mcu_common/common_data.h>
#include <mcu_common/serial_node.h>
class ForceControl
{
public:
    ForceData force_data;                           // 接收数据

    ForceDataStruct cmd_data;                         // 发送数据
    dataScope port_manager;                    // 串口管理对象
    SerialPort force_port;    // 串口对象

    ForceControl();                     // 构造函数
    ~ForceControl(){};                                  // 析构函数
    void set_port(int baud_rate, const std::string& _name);
private:
    ros::NodeHandle node_handle_;
};

