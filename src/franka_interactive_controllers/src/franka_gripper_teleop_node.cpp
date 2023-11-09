#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <cstdlib>  // for system()
static bool in_open_state = false;
static bool in_close_state = false;

void cmdCallback(const std_msgs::Int32::ConstPtr& msg) {
    if(msg->data == 2 && in_open_state ==false) {
        system("rosrun franka_interactive_controllers libfranka_gripper_run 1");
        ROS_INFO("Executed command for cmd=1"); 
        in_open_state = true;
        in_close_state = false;
    } else if(msg->data == 3 && in_close_state ==false) {
        system("rosrun franka_interactive_controllers libfranka_gripper_run 0");
        ROS_INFO("Executed command for cmd=2");
        in_close_state = true;
        in_open_state = false;
    }else
    {
        in_open_state = false;
        in_close_state = false;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "cmd_executor_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/end_effector_teleop", 10, cmdCallback);
    ROS_INFO("Executed command for end_effector_teleop");

    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        ros::spinOnce(); 
        loop_rate.sleep();
    }
    return 0;
}
