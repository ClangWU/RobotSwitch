#pragma once
#include "ros/ros.h"
#include <vector>

#pragma pack(1)
struct ForceDataStruct
{
    float force_send_data;
};
#pragma pack()

struct ForceData
{
    float _force;  
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
    }else{
        ROS_ERROR_STREAM("Unable to read data ");
    }
  }
};