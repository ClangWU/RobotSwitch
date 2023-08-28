// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
// #include <franka_interactive_controllers/cartesian_pose_example_controller.h>
#include <cartesian_pose_node_controller.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <controller_interface/controller_base.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_reactive_controller
{

  bool CartesianPoseNodeController::init(hardware_interface::RobotHW *robot_hardware,
                                         ros::NodeHandle &node_handle)
  {

    cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
    if (cartesian_pose_interface_ == nullptr)
    {
      ROS_ERROR(
          "CartesianPoseNodeController: Could not get Cartesian Pose "
          "interface from hardware");
      return false;
    }

    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id))
    {
      ROS_ERROR("CartesianPoseNodeController: Could not get parameter arm_id");
      return false;
    }

    try
    {
      cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
          cartesian_pose_interface_->getHandle(arm_id + "_robot"));
    }
    catch (const hardware_interface::HardwareInterfaceException &e)
    {
      ROS_ERROR_STREAM(
          "CartesianPoseNodeController: Exception getting Cartesian handle: " << e.what());
      return false;
    }

    auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr)
    {
      ROS_ERROR("CartesianPoseNodeController: Could not get state interface from hardware");
      return false;
    }

    try
    {
      auto state_handle = state_interface->getHandle(arm_id + "_robot");

      std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
      for (size_t i = 0; i < q_start.size(); i++)
      {
        if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1)
        {
          ROS_ERROR_STREAM(
              "CartesianPoseNodeController: Robot is not in the expected starting position for "
              "running this example. Run `roslaunch franka_reactive_controllers move_to_start.launch "
              "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
          return false;
        }
      }
    }
    catch (const hardware_interface::HardwareInterfaceException &e)
    {
      ROS_ERROR_STREAM(
          "CartesianPoseNodeController: Exception getting state handle: " << e.what());
      return false;
    }

    // Rate Limiting
    if (!node_handle.getParam("rate_limiting/linear/velocity", max_translational_velocity))
    {
      ROS_ERROR("CartesianVelocityNodeController: Could not get parameter rate_limiting/linear/velocity");
      return false;
    }
    if (!node_handle.getParam("rate_limiting/linear/acceleration", max_translational_acceleration))
    {
      ROS_ERROR("CartesianVelocityNodeController: Could not get parameter rate_limiting/acc/acceleration");
      return false;
    }
    if (!node_handle.getParam("rate_limiting/linear/jerk", max_translational_jerk))
    {
      ROS_ERROR("CartesianVelocityNodeController: Could not get parameter rate_limiting/linear/jerk");
      return false;
    }
    if (!node_handle.getParam("rate_limiting/angular/velocity", max_rotational_velocity))
    {
      ROS_ERROR("CartesianVelocityNodeController: Could not get parameter rate_limiting/angular/velocity");
      return false;
    }
    if (!node_handle.getParam("rate_limiting/angular/acceleration", max_rotational_acceleration))
    {
      ROS_ERROR("CartesianVelocityNodeController: Could not get parameter rate_limiting/acc/acceleration");
      return false;
    }
    if (!node_handle.getParam("rate_limiting/angular/jerk", max_rotational_jerk))
    {
      ROS_ERROR("CartesianVelocityNodeController: Could not get parameter rate_limiting/angular/jerk");
      return false;
    }

    node_handle.param<bool>("stop_on_contact", stop_on_contact, true);

    pose_command_subscriber = node_handle.subscribe("/cartesian_pose_node_controller/cartesian_pose",
                                                    10,
                                                    &CartesianPoseNodeController::cartesian_pose_callback,
                                                    this);
    ROS_INFO("\033[1;31mpose_command_subscriber  initialize\033[0m");
    return true;                        
  }

  void CartesianPoseNodeController::starting(const ros::Time & /* time */)
  {
    initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d; // O_T_EE_d
    pose_command_ =  initial_pose_;
    new_pose_ = initial_pose_;
    rotation_mat << initial_pose_[0], initial_pose_[1], initial_pose_[2], 
                    initial_pose_[4], initial_pose_[5], initial_pose_[6], 
                    initial_pose_[8], initial_pose_[9], initial_pose_[10];
    std::cout << "\033[34mO_T_EE_d:\033[0m"  << std::endl << rotation_mat << std::endl;
    std::cout << " x " << initial_pose_[12] << " y " << initial_pose_[13] << " z " << initial_pose_[14] << std::endl;
    pose_command_mat = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(initial_pose_.data());
    Eigen::Quaterniond quaternion(rotation_mat);
    std::cout << "pose_command_mat: " << std::endl << pose_command_mat << std::endl;

    std::cout << "四元数:\n"
              << "w: " << quaternion.w() << "\n"
              << "x: " << quaternion.x() << "\n"
              << "y: " << quaternion.y() << "\n"
              << "z: " << quaternion.z() << std::endl;

    //   机械臂四元数:
    // w: -4.92282e-05
    // x: 0.999998
    // y: -0.000260781
    // z: -0.000265499
    std::copy(pose_command_mat.data(), pose_command_mat.data() + pose_command_mat.size(), initial_pose_.begin());
    std::cout << "initial_pose_:" << std::endl;
    for (size_t row = 0; row < 4; ++row) {
      for (size_t col = 0; col < 4; ++col) {
        std::cout << initial_pose_[col * 4 + row] << " ";
      }
      std::cout << std::endl;
    }
    time_since_last_command = ros::Duration(0.0);
  }
void CartesianPoseNodeController::cartesian_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
{
    if(pose_msg->pose.position.y > INTERACT_POSE_POS) {
      pos_y_ = DELTA_POSE_TRANS;
    }else if(pose_msg->pose.position.y < -INTERACT_POSE_NEG){
      pos_y_ = -DELTA_POSE_TRANS;
    }else{
      pos_y_ = 0;
    }
    if(pose_msg->pose.position.x > MOVE_X_POSE_POS) {
      pos_x_ = DELTA_POSE_TRANS;
    }else if(pose_msg->pose.position.x < -MOVE_X_POSE_NEG){
      pos_x_ = -DELTA_POSE_TRANS;
    }else{
      pos_x_ = 0;
    }
      if(pose_msg->pose.position.z > MOVE_Z_POSE_POS) {
        pos_z_ = DELTA_POSE_TRANS;
      }else if(pose_msg->pose.position.z < -MOVE_Z_POSE_NEG){
        pos_z_ = -DELTA_POSE_TRANS;
      }else{
        pos_z_ = 0;
      }

  Eigen::Quaterniond quat(pose_msg->pose.orientation.w,
                          pose_msg->pose.orientation.x,
                          pose_msg->pose.orientation.y,
                          pose_msg->pose.orientation.z);

  rotation_mat = quat.toRotationMatrix();

      // std::cout << "rotation_mat:" << std::endl << rotation_mat << std::endl;

  // pose_command_
      // std::cout << "pose_command_:" << std::endl;
    // for (size_t row = 0; row < 4; ++row) {
    //   for (size_t col = 0; col < 4; ++col) {
    //     std::cout << pose_command_[col * 4 + row] << " ";
    //   }
    //   std::cout << std::endl;
    // }
  time_since_last_command = ros::Duration(0.0);
}

void CartesianPoseNodeController::update(const ros::Time & /* time */,
                                         const ros::Duration &period)
{

  // TODO: Change this code to take a desired pose message stamped
  time_since_last_command += period;
  auto robot_state_ = cartesian_pose_handle_->getRobotState();

  // if (time_since_last_command.toSec() > max_duration_between_commands)
  // {
  //     pose_command_ =  robot_state_.O_T_EE_c;
  //   // pose_command_ =  robot_state_.O_T_EE;
  // }else{
    pose_command_mat.topLeftCorner<3,3>() = rotation_mat;
    // pose_command_ 
    std::copy(pose_command_mat.data(), pose_command_mat.data() + pose_command_mat.size(), 
          pose_command_.begin());
  // }
    // std::cout << "\033[34mpose_command_:\033[0m" << std::endl;

    // for (size_t row = 0; row < 4; ++row) {
    //   for (size_t col = 0; col < 4; ++col) {
    //     std::cout << pose_command_[col * 4 + row] << " ";
    //   }
    //   std::cout << std::endl;
    // }

    // new_pose_ = franka::limitRate(
    //             max_translational_velocity,
    //             max_translational_acceleration,
    //             max_translational_jerk,
    //             max_rotational_velocity,
    //             max_rotational_acceleration,
    //             max_rotational_jerk,
    //             pose_command_,// pose_command_
    //             robot_state_.O_T_EE_c,
    //             robot_state_.O_dP_EE_c,
    //             robot_state_.O_ddP_EE_c);
  double radius = 0.30;
  double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * time_since_last_command.toSec()));
  double delta_x = radius * std::sin(angle);
  double delta_z = radius * (std::cos(angle) - 1);
  std::array<double, 16> new_pose = initial_pose_;

  // Eigen::Quaterniond quat0(0.000282, 0.9997, -0.024037, -0.000130);

  // Eigen::Matrix3d rotation0 = quat0.toRotationMatrix();
  // new_pose[0] = rotation0(0,0);
  // new_pose[1] = rotation0(1,0);
  // new_pose[2] = rotation0(2,0);
  // new_pose[4] = rotation0(0,1);
  // new_pose[5] = rotation0(1,1);
  // new_pose[6] = rotation0(2,1);
  // new_pose[8] = rotation0(0,2);
  // new_pose[9] = rotation0(1,2);
  // new_pose[10] = rotation0(2,2);

  new_pose[12] -= delta_x;
  new_pose[14] -= delta_z;
  cartesian_pose_handle_->setCommand(new_pose);
    // cartesian_pose_handle_->setCommand(pose_command_);
}

void CartesianPoseNodeController::stopping(const ros::Time & /*time*/)
{
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

} // namespace franka_interactive_controllers

PLUGINLIB_EXPORT_CLASS(franka_reactive_controller::CartesianPoseNodeController,
                       controller_interface::ControllerBase)
