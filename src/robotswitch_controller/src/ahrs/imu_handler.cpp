#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/thread/thread.hpp>
#include <iostream>

void multi_callback(const sensor_msgs::ImuConstPtr& imu1, const sensor_msgs::ImuConstPtr& imu2)
{
    std::cout << "两个IMU消息已同步！" << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_sync_node");
    ros::NodeHandle n;

    // 订阅第一个IMU话题
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub1(n, "imu_topic1", 1000, ros::TransportHints().tcpNoDelay());

    // 订阅第二个IMU话题
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub2(n, "imu_topic2", 1000, ros::TransportHints().tcpNoDelay());

    // 定义同步策略
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, sensor_msgs::Imu> syncPolicy;

    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), imu_sub1, imu_sub2);

    // 注册回调
    sync.registerCallback(boost::bind(&multi_callback, _1, _2));

    std::cout << "正在等待同步的IMU消息..." << std::endl;

    ros::spin();

    return 0;
}

