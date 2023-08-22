#pragma once
#include <rs_common/common_data.h>
#include <rs_common/serial_node.h>
class LeftControl
{
public:
    LeftData left_data;                           // 接收数据

    dataScope port_manager;                    // 串口管理对象
    SerialPort left_port;    // 串口对象

    LeftControl();                     // 构造函数
    ~LeftControl(){};                                  // 析构函数
    void set_port(int baud_rate, const std::string& _name);
private:
    ros::NodeHandle node_handle_;
    ros::Subscriber key_suber_;
};

