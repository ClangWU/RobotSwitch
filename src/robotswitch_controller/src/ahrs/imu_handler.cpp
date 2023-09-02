#include <ahrs/imu_handler.h>
namespace RobotSwitch
{
 IMU_Handler::IMU_Handler()
    : nh_(), private_nh_("~"), upperarm_sub_(nh_, "/upperarm/imu", 1000, ros::TransportHints().tcpNoDelay()),
      forearm_sub_(nh_, "/forearm/imu", 1000, ros::TransportHints().tcpNoDelay()),
      sync_(syncPolicy(10), upperarm_sub_, forearm_sub_)
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

        sync_.registerCallback(boost::bind(&IMU_Handler::multi_callback, this, _1, _2));
    }
    void IMU_Handler::multi_callback(const sensor_msgs::ImuConstPtr& upper_imu, const sensor_msgs::ImuConstPtr& fore_imu)
    {
        qA = Eigen::Quaterniond(upper_imu->orientation.w, upper_imu->orientation.x, upper_imu->orientation.y, upper_imu->orientation.z);
        qB = Eigen::Quaterniond(fore_imu->orientation.w, fore_imu->orientation.x, fore_imu->orientation.y, fore_imu->orientation.z);
        computeTransform();
    }
    void IMU_Handler::computeTransform()
    {
        pA = qA._transformVector(lA);
        pB = qB._transformVector(lB);
        pF = pA + pB;
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
        fore_pose.pose.position.x = pF(0);
        fore_pose.pose.position.y = pF(1);
        fore_pose.pose.position.z = pF(2);
        fore_pose.pose.orientation.x = qF.x();
        fore_pose.pose.orientation.y = qF.y();
        fore_pose.pose.orientation.z = qF.z();
        fore_pose.pose.orientation.w = qF.w();

        fore_pose_publisher.publish(fore_pose);
    }
} // namespace RobotSwitch

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_handler_node");
    RobotSwitch::IMU_Handler imu_handler_node;
    imu_handler_node.run();
    return 0;
}

