#pragma once
#include <serial/serial.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

using namespace std;

class SerialPort
{
private:
    serial::Serial* serial_ptr = nullptr;
    serial::Serial ser; //声明串口对象
    std::string port_name;
public:
    SerialPort(int baud_rate = 115200, const std::string& _name = "/dev/ttyUSB0"){
    port_name = _name;
    set_port_info(baud_rate, _name);
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    ser.setTimeout(to);
    serial_ptr = &ser;
  }
    ~SerialPort(){};
    void set_port_info(int baud_rate, const std::string& _name){
        ser.setPort(_name); //你的串口设备号，这是一个示例，你需要替换为你的设备号
        ser.setBaudrate(baud_rate); //设置波特率
    }
    void open(){
        try{
            serial_ptr->open();
        }
        catch (serial::IOException& e){
        ROS_ERROR_STREAM("Unable to open port ");
        }
    }
    void close(){
        //检查串口是否打开成功
        if(serial_ptr->isOpen()){
            ROS_INFO_STREAM("Serial Port initialized");
        } 
        else{
            ROS_ERROR_STREAM("Unable to initialized ");
        }
    }

    bool isOpen() { 
        if(serial_ptr->isOpen()){
            // ROS_INFO_NAMED("arm_log", "Successfully open serial port %s", port_name.c_str());
            return true;
        }
     };

    template <typename T>
    std::vector<T> readStruct(unsigned char head, unsigned char tail)
    {
    std::vector<T> vec_t;
    const int LENGTH = 20;
    const int SIZE = sizeof(T);
    unsigned char read_buffer[LENGTH] = {0};
    size_t len_result = serial_ptr->read(read_buffer, LENGTH);
    for (size_t i = 0; (i + SIZE + 1) < len_result; i++)
    {
        if (read_buffer[i] == head && read_buffer[i + SIZE + 1] == tail)
        { 
        //TODO 引发 GCC 警告: -Wstrict-aliasing
        vec_t.push_back(*(reinterpret_cast<T *>(&read_buffer[i + 1])));
        }
    }
    return vec_t;
    }

    template <typename T>
    bool writeStruct(T data_struct)
    {
        size_t len_result = serial_ptr->write(reinterpret_cast<const uint8_t*>(&data_struct), sizeof(data_struct));
        return (sizeof(data_struct) == len_result);
    }
};
