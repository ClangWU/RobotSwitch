#pragma once
#include "ros/ros.h"
#include <vector>
#include <bits/types.h>
#include <memory>

#pragma pack(1)
struct ForceData
{
    float _force_y;
    float _force_z;  
};
#pragma pack()

struct TeleopData
{
    int _start;
    int _grip;  
} __attribute__((packed));

struct IMUData
{
    float qw;  
    float qx;  
    float qy;  
    float qz;  
} __attribute__((packed));

struct LeftData
{
    int _y;
} __attribute__((packed));

struct RightData
{
    int _x;
    int _z;
} __attribute__((packed));

class dataScope
{
public:
    dataScope(){};                               // 构造函数
    int wrong_collect = 0;
    
    template <typename T>
    T filter(const std::vector<T> &data){
    if (!data.empty())
    {
        return data.back();
    }
    // else{
    //     ROS_ERROR_STREAM("Unable to read data ");
    // }
  }
};