// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <memory>
#include <string>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Geometry>
#include <franka/rate_limiting.h>
#include <geometry_msgs/PoseStamped.h>
#include <franka_hw/franka_cartesian_command_interface.h>

namespace franka_reactive_controller {

class CartesianPoseNodeController
    : public controller_interface::MultiInterfaceController<franka_hw::FrankaPoseCartesianInterface,
                                                            franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void stopping(const ros::Time&) override;
  void cartesian_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);

 private:
  franka_hw::FrankaPoseCartesianInterface* cartesian_pose_interface_;
  std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;
  ros::Duration time_since_last_command;
  std::array<double, 16> initial_pose_;
  std::array<double, 16> last_pose_;
  std::array<double, 16> new_pose_;
  std::array<double, 16> pose_command;
  Eigen::Matrix4d pose_command_mat;

  double max_duration_between_commands;
  double max_translational_velocity;
  double max_translational_acceleration;
  double max_translational_jerk;
  double max_rotational_velocity;
  double max_rotational_acceleration;
  double max_rotational_jerk;
  bool stop_on_contact;
};

}  // namespace franka_reactive_controllers
