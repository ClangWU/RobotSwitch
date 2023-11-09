#include <ros/ros.h>
#include "std_msgs/Int32.h"
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <Eigen/Dense>
#include <iostream>

namespace RobotSwitch
{
class IMU_Handler
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher pose_publisher;
    ros::Publisher  upper_pose_publisher;
    ros::Publisher  fore_pose_publisher;
    ros::Publisher  hand_pose_publisher;
    message_filters::Subscriber<sensor_msgs::Imu> upperarm_sub_;
    message_filters::Subscriber<sensor_msgs::Imu> forearm_sub_;
    message_filters::Subscriber<sensor_msgs::Imu> hand_sub_;
    ros::Subscriber cmd_subscriber;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, sensor_msgs::Imu, sensor_msgs::Imu> syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync_;

    Eigen::Quaterniond qA, qB, qC, qB2A, qF, calibration_qA, calibration_qB, calibration_qC;
    double fore_len = 0.22; // default values can be changed
    double upper_len = 0.28; // default values can be changed
    Eigen::Vector3d pA, pB, pF, pCal,lA, lB;
    bool initialized = false;
    int _cmd;
public:
   IMU_Handler();
    ~IMU_Handler(){};
    void multi_callback(const sensor_msgs::ImuConstPtr& upper_imu, 
                        const sensor_msgs::ImuConstPtr& fore_imu, 
                        const sensor_msgs::ImuConstPtr& hand_imu);
    void computeTransform();
    void run()
    {        
        std::cout << "正在等待同步的IMU消息..." << std::endl;
        ros::spin();
    }
};

} // namespace RobotSwitch
