#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
// 假设autobot.h存在且定义了Autocut类
#include "autobot.h"
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <chrono>
#include <thread>

// 使用vector来自动管理内存
static std::vector<float> state;
static std::vector<float> act = {0, 0, 0}; 

// 假设Autocut类有合适的接口
static Autocut obj;

void ObsCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    state.clear();
    for (auto val : msg->data) {
        state.push_back(val);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "autobot"); // 初始化节点
    ros::NodeHandle nh;
    ros::Publisher robot_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/cartesian_impedance_controller/desired_pose", 10);
    ros::Subscriber obs_sub = nh.subscribe("/observation", 10, ObsCallback);
    ros::Rate loop_rate(10); // 设置循环频率
    
    while (ros::ok()) {            
        // 假设update方法现在接受std::vector<float>类型
        // 并且我们有一个方法来生成所需的PoseStamped消息
        obj.update(state, act);
        robot_pose_publisher.publish(desired_pose);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
