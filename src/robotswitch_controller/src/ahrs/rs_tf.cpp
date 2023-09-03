#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

// Global variables to store the current pose values
geometry_msgs::Pose upperarmPose;
geometry_msgs::Pose forearmPose;

// Callbacks
void upperarmCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    upperarmPose = msg->pose;
}

void forearmCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    forearmPose = msg->pose;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "arm_tf_publisher");
    ros::NodeHandle node;

    // Subscribers
    ros::Subscriber upperarmSub = node.subscribe("upperarm/pose", 10, upperarmCallback);
    ros::Subscriber forearmSub = node.subscribe("forearm/pose", 10, forearmCallback);

    tf2_ros::TransformBroadcaster tf_broadcaster;
    ros::Rate rate(200);  

    while(node.ok()){
        ros::spinOnce();

        geometry_msgs::TransformStamped upper_arm_tf, forearm_tf;

        // Upper arm TF (Shoulder to Elbow)
        upper_arm_tf.header.stamp = ros::Time::now();
        upper_arm_tf.header.frame_id = "shoulder_frame"; // This is your fixed frame
        upper_arm_tf.child_frame_id = "elbow_frame"; // Upper arm's end

        upper_arm_tf.transform.translation.x = upperarmPose.position.x;
        upper_arm_tf.transform.translation.y = upperarmPose.position.y;
        upper_arm_tf.transform.translation.z = upperarmPose.position.z;

        upper_arm_tf.transform.rotation = upperarmPose.orientation;
        tf_broadcaster.sendTransform(upper_arm_tf);

        // Forearm TF (Elbow to Wrist)
        forearm_tf.header.stamp = ros::Time::now();
        forearm_tf.header.frame_id = "elbow_frame"; // Start from the end of the upper arm
        forearm_tf.child_frame_id = "wrist_frame"; // Forearm's end

        forearm_tf.transform.translation.x = forearmPose.position.x;
        forearm_tf.transform.translation.y = forearmPose.position.y;
        forearm_tf.transform.translation.z = forearmPose.position.z;

        forearm_tf.transform.rotation = forearmPose.orientation;

        // Broadcasting the transforms
        tf_broadcaster.sendTransform(forearm_tf);

        rate.sleep();
    }
    return 0;
}
