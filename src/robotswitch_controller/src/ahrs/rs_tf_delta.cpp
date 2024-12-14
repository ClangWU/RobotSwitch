#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

// Global variables to store the current pose values
geometry_msgs::Pose deltaPose;
geometry_msgs::Pose handPose;

// Callbacks
void handCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    handPose = msg->pose;
}
void deltaCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    deltaPose = msg->pose;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "delta_tf_publisher");
    ros::NodeHandle node;

    // Subscribers
    ros::Subscriber deltaSub = node.subscribe("/cartesian_impedance_controller/desired_pose", 10, deltaCallback);
    ros::Subscriber handSub  = node.subscribe("/arm_pose", 10, handCallback);

    tf2_ros::TransformBroadcaster tf_broadcaster;
    ros::Rate rate(200);  

    while(node.ok()){
        ros::spinOnce();
        geometry_msgs::TransformStamped delta_tf, hand_tf;

        // Upper arm TF (Shoulder to Elbow)
        // delta_tf.header.stamp = ros::Time::now();
        // delta_tf.header.frame_id = "base_frame"; // This is your fixed frame
        // delta_tf.child_frame_id = "delta_frame"; // Upper arm's end

        // delta_tf.transform.translation.x = deltaPose.position.x;
        // delta_tf.transform.translation.y = deltaPose.position.y;
        // delta_tf.transform.translation.z = deltaPose.position.z;
        // delta_tf.transform.rotation = deltaPose.orientation;
        // tf_broadcaster.sendTransform(delta_tf);

        // Forearm TF (Elbow to Wrist)
        hand_tf.header.stamp = ros::Time::now();
        hand_tf.header.frame_id = "base_frame"; // Start from the end of the fore arm
        hand_tf.child_frame_id = "hand_frame"; // Hand's end
        hand_tf.transform.translation.x = handPose.position.x;
        hand_tf.transform.translation.y = handPose.position.y;
        hand_tf.transform.translation.z = handPose.position.z;
        hand_tf.transform.rotation = handPose.orientation;

        // Broadcasting the transforms
        tf_broadcaster.sendTransform(hand_tf);

        rate.sleep();
    }
    return 0;
}
