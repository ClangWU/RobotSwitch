#pragma once
#include <mcu_common/common_data.h>
#include <mcu_common/serial_node.h>
class RightControl
{
public:
    RightData right_data;                           // 接收数据

    dataScope port_manager;                    // 串口管理对象
    SerialPort port;    // 串口对象

    RightControl();                     // 构造函数
    ~RightControl(){};                                  // 析构函数
    void set_port(int baud_rate, const std::string& _name);

private:
    ros::NodeHandle node_handle_;
};

