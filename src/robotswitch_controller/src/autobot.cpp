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
static geometry_msgs::PoseStamped cut_pose;

// 假设Autocut类有合适的接口
static Autocut obj;

void ObsCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    state.clear();
    for (auto val : msg->data) {
        state.push_back(val);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "autobot_node"); // 初始化节点
    ros::NodeHandle nh;
    ros::Publisher robot_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/cartesian_impedance_controller/desired_pose", 10);
    ros::Publisher autocut_publisher = nh.advertise<std_msgs::Float32MultiArray>("/autocut", 10);
    ros::Subscriber obs_sub = nh.subscribe("/observation", 10, ObsCallback);
    ros::Rate loop_rate(50); // 设置循环频率
    std_msgs::Float32MultiArray obs_array;

    double roll, pitch, yaw = 0.0;
    double cy, sy, cp, sp, cr, sr;
    while (ros::ok()) {            
        // 假设update方法现在接受std::vector<float>类型
        // 并且我们有一个方法来生成所需的PoseStamped消息
        obj.update(state, act);

        roll = (act[2] - 180.0) * M_PI / 180.0;
        // roll = (0 - 180.0) * M_PI / 180.0;

        pitch = 20 / 180.0;
        yaw = 0.0;
        cy = cos(yaw * 0.5);
        sy = sin(yaw * 0.5);
        cp = cos(pitch * 0.5);
        sp = sin(pitch * 0.5);
        cr = cos(roll * 0.5);
        sr = sin(roll * 0.5);

        cut_pose.pose.position.x = 0;
        cut_pose.pose.position.y = act[0];
        cut_pose.pose.position.z = act[1];
        // cut_pose.pose.position.y = 0;
        // cut_pose.pose.position.z = 0;
        cut_pose.pose.orientation.w = cr * cp * cy + sr * sp * sy;
        cut_pose.pose.orientation.x = sr * cp * cy - cr * sp * sy;
        cut_pose.pose.orientation.y = cr * sp * cy + sr * cp * sy;
        cut_pose.pose.orientation.z = cr * cp * sy - sr * sp * cy;

        robot_pose_publisher.publish(cut_pose);

        obs_array.data.clear();
        obs_array.data.push_back(act[0]*100);
        obs_array.data.push_back(act[1]*100);
        obs_array.data.push_back(act[2]);
        if (state.size() < 6) {
            std::cout << "obs size" << state.size() << std::endl;
            std::cerr << "Invalid observation data." << std::endl;} // 或者其他错误处理
        else{
            obs_array.data.push_back(state[0]*100);
            obs_array.data.push_back(state[1]*100);
            obs_array.data.push_back(state[2]);
            obs_array.data.push_back(state[3]);
            obs_array.data.push_back(state[4]);
        }
        autocut_publisher.publish(obs_array);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
