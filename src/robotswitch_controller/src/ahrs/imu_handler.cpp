#include <ahrs/imu_handler.h>
namespace RobotSwitch
{
 IMU_Handler::IMU_Handler()
    : nh_(), private_nh_("~"), upperarm_sub_(nh_, "/upperarm/imu", 1000, ros::TransportHints().tcpNoDelay()),
      forearm_sub_(nh_, "/forearm/imu", 1000, ros::TransportHints().tcpNoDelay()),
      hand_sub_(nh_, "/hand/imu", 1000, ros::TransportHints().tcpNoDelay()),
      sync_(syncPolicy(10), upperarm_sub_, forearm_sub_, hand_sub_)
    {
        if (nh_.getParam("handler_config/forearm_length", fore_len)) {
        ROS_INFO_STREAM("\033[1;32mGot forearm_length : " << fore_len << "\033[0m");
        } else{
            ROS_WARN("Failed to get forearm_length from the parameter server! Using default value.");
        }
        if (nh_.getParam("handler_config/upperarm_length", upper_len)) {
        ROS_INFO_STREAM("\033[1;32mGot upperarm_length : " << upper_len << "\033[0m");
        }else{
            ROS_WARN("Failed to get upperarm_length from the parameter server! Using default value.");
        }
        lA <<  upper_len, 0, 0;
        lB <<  fore_len , 0,0;

        pose_publisher = nh_.advertise<geometry_msgs::PoseStamped>("/cartesian_impedance_controller/desired_pose", 10);
        upper_pose_publisher = nh_.advertise<geometry_msgs::PoseStamped>("upperarm/pose", 10);
        fore_pose_publisher = nh_.advertise<geometry_msgs::PoseStamped>("forearm/pose", 10);
        hand_pose_publisher = nh_.advertise<geometry_msgs::PoseStamped>("hand/pose", 10);

        sync_.registerCallback(boost::bind(&IMU_Handler::multi_callback, this, _1, _2, _3));
    }
    void IMU_Handler::multi_callback(const sensor_msgs::ImuConstPtr& upper_imu, const sensor_msgs::ImuConstPtr& fore_imu, const sensor_msgs::ImuConstPtr& hand_imu)
    {
            // ROS_WARN("Failed to get upperarm_length from the parameter server! Using default value.");
        if (!initialized) {
            Eigen::Quaterniond  qA_Raw = Eigen::Quaterniond(upper_imu->orientation.w, upper_imu->orientation.x, upper_imu->orientation.y, upper_imu->orientation.z);
            Eigen::Quaterniond  qB_Raw = Eigen::Quaterniond(fore_imu->orientation.w, fore_imu->orientation.x, fore_imu->orientation.y, fore_imu->orientation.z);
            Eigen::Quaterniond  qC_Raw = Eigen::Quaterniond(hand_imu->orientation.w, hand_imu->orientation.x, hand_imu->orientation.y, hand_imu->orientation.z);

            Eigen::Quaterniond robot_arm_initial_quaternion(0.7071,0,0.7071,0);
            Eigen::Quaterniond robot_hand_initial_quaternion(0.0, 1.0, 0, 0);
            calibration_qA = robot_arm_initial_quaternion * qA_Raw.conjugate();
            calibration_qB = robot_arm_initial_quaternion * qB_Raw.conjugate();
            calibration_qC = robot_hand_initial_quaternion * qC_Raw.conjugate();
            pCal = 
            robot_arm_initial_quaternion._transformVector(lA)+ robot_arm_initial_quaternion._transformVector(lB);
            initialized = true;
        }else
        {
            qA = Eigen::Quaterniond(upper_imu->orientation.w, upper_imu->orientation.x, upper_imu->orientation.y, upper_imu->orientation.z);
            qB = Eigen::Quaterniond(fore_imu->orientation.w, fore_imu->orientation.x, fore_imu->orientation.y, fore_imu->orientation.z);
            qC = Eigen::Quaterniond(hand_imu->orientation.w, hand_imu->orientation.x, hand_imu->orientation.y, hand_imu->orientation.z);

            // 计算将 qA_Raw 对齐到 robot_arm_initial_quaternion 的校准四元数
            qA = calibration_qA * qA;
            // 对 qB 同样的处理
            qB = calibration_qB * qB;
            qC = calibration_qC * qC;
            computeTransform();
        }
    }
    void IMU_Handler::computeTransform()
    {
        qB2A = qA.inverse() * qB;
        pA = qA._transformVector(lA);
        pB = qB._transformVector(lB);
        pF = pA + pB - pCal;//算出基于初始化坐标的位移变量
        qF = qA * qB;

        geometry_msgs::PoseStamped upper_pose;
        upper_pose.pose.position.x = pA(0);
        upper_pose.pose.position.y = pA(1);
        upper_pose.pose.position.z = pA(2);
        upper_pose.pose.orientation.x = qA.x();
        upper_pose.pose.orientation.y = qA.y();
        upper_pose.pose.orientation.z = qA.z();
        upper_pose.pose.orientation.w = qA.w();

        upper_pose_publisher.publish(upper_pose);

        geometry_msgs::PoseStamped fore_pose;
        fore_pose.pose.position.x = pB(0);
        fore_pose.pose.position.y = pB(1);
        fore_pose.pose.position.z = pB(2);
        fore_pose.pose.orientation.x = qB.x();
        fore_pose.pose.orientation.y = qB.y();
        fore_pose.pose.orientation.z = qB.z();
        fore_pose.pose.orientation.w = qB.w();

        fore_pose_publisher.publish(fore_pose);

        geometry_msgs::PoseStamped hand_pose;
        hand_pose.pose.position.x = 0;
        hand_pose.pose.position.y = 0;
        hand_pose.pose.position.z = 0;
        hand_pose.pose.orientation.x = qC.x();
        hand_pose.pose.orientation.y = qC.y();
        hand_pose.pose.orientation.z = qC.z();
        hand_pose.pose.orientation.w = qC.w();

        hand_pose_publisher.publish(hand_pose);

        geometry_msgs::PoseStamped _pose;
        _pose.pose.position.x = pF(0);
        _pose.pose.position.y = pF(1);
        _pose.pose.position.z = pF(2);
        _pose.pose.orientation.x = qC.x();
        _pose.pose.orientation.y = qC.y();
        _pose.pose.orientation.z = qC.z();
        _pose.pose.orientation.w = qC.w();
        pose_publisher.publish(_pose);
    }
} // namespace RobotSwitch

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_handler_node");
    RobotSwitch::IMU_Handler imu_handler_node;
    imu_handler_node.run();
    return 0;
}

