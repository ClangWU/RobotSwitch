#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

// Global variables to store the current pose values
geometry_msgs::Pose imuPose;

// Callbacks
void imuCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    imuPose = msg->pose;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "imu_tf_publisher");
    ros::NodeHandle node;

    // Subscribers
    ros::Subscriber _imu = node.subscribe("/hand_IMU", 10, imuCallback);

    tf2_ros::TransformBroadcaster tf_broadcaster;
    // ros::Rate rate(200);  

    while(node.ok()){
        ros::spinOnce();

        geometry_msgs::TransformStamped imu_tf;

        // Upper arm TF (Shoulder to Elbow)
        imu_tf.header.stamp = ros::Time::now();
        imu_tf.header.frame_id = "base_frame"; // This is your fixed frame
        imu_tf.child_frame_id = "imu_frame"; // Upper arm's end

        imu_tf.transform.translation.x = imuPose.position.x;
        imu_tf.transform.translation.y = imuPose.position.y;
        imu_tf.transform.translation.z = imuPose.position.z;

        imu_tf.transform.rotation = imuPose.orientation;
        tf_broadcaster.sendTransform(imu_tf);

        // rate.sleep();
    }
    return 0;
}
