// This code was derived from franka_example controllers
// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license.
// Current development and modification of this code by Nadia Figueroa (MIT) 2021.

#include <cartesian_pose_impedance_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <pseudo_inversion.h>
#include <hardware_interface/joint_command_interface.h>

namespace franka_interactive_controllers {

bool CartesianPoseImpedanceController::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {
  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;

  sub_desired_pose_ = node_handle.subscribe(
      "/cartesian_impedance_controller/desired_pose", 20, &CartesianPoseImpedanceController::desiredPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  pub_observation = node_handle.advertise<std_msgs::Float32MultiArray>("/observation", 1000);
  // Getting ROSParams
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("CartesianPoseImpedanceController: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "CartesianPoseImpedanceController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  // Getting libranka control interfaces
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianPoseImpedanceController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianPoseImpedanceController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianPoseImpedanceController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianPoseImpedanceController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianPoseImpedanceController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "CartesianPoseImpedanceController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  // Getting Dynamic Reconfigure objects
  dynamic_reconfigure_compliance_param_node_ =
      ros::NodeHandle(node_handle.getNamespace() + "dynamic_reconfigure_compliance_param_node");

  dynamic_server_compliance_param_ = std::make_unique<
      dynamic_reconfigure::Server<franka_interactive_controllers::minimal_compliance_paramConfig>>(

      dynamic_reconfigure_compliance_param_node_);
  dynamic_server_compliance_param_->setCallback(
      boost::bind(&CartesianPoseImpedanceController::complianceParamCallback, this, _1, _2));


  // Initializing variables
  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  ///////////////////////////////////////////////////////////////////////////
  ////////////////  Parameter Initialization from YAML FILES!!!     /////////
  ///////////////////////////////////////////////////////////////////////////

   // Initialize stiffness and damping gains
  cartesian_stiffness_target_.setIdentity();
  cartesian_damping_target_.setIdentity();
  std::vector<double> cartesian_stiffness_target_yaml;
  if (!node_handle.getParam("cartesian_stiffness_target", cartesian_stiffness_target_yaml) || cartesian_stiffness_target_yaml.size() != 6) {
    ROS_ERROR(
      "CartesianPoseImpedanceController: Invalid or no cartesian_stiffness_target_yaml parameters provided, "
      "aborting controller init!");
    return false;
  }
  for (int i = 0; i < 6; i ++) {
    cartesian_stiffness_target_(i,i) = cartesian_stiffness_target_yaml[i];
  }
  // Damping ratio = 1
  default_cart_stiffness_target_ << 300, 300, 300, 50, 50, 50;
  for (int i = 0; i < 6; i ++) {      
    if (cartesian_stiffness_target_yaml[i] == 0.0)
      cartesian_damping_target_(i,i) = 2.0 * sqrt(default_cart_stiffness_target_[i]);
    else
      cartesian_damping_target_(i,i) = 2.0 * sqrt(cartesian_stiffness_target_yaml[i]);
  }
  //ROS_INFO_STREAM("cartesian_stiffness_target_: " << std::endl <<  cartesian_stiffness_target_);
  //ROS_INFO_STREAM("cartesian_damping_target_: " << std::endl <<  cartesian_damping_target_);

  if (!node_handle.getParam("nullspace_stiffness", nullspace_stiffness_target_) || nullspace_stiffness_target_ <= 0) {
    ROS_ERROR(
      "CartesianPoseImpedanceController: Invalid or no nullspace_stiffness parameters provided, "
      "aborting controller init!");
    return false;
  }
  //ROS_INFO_STREAM("nullspace_stiffness_target_: " << std::endl <<  nullspace_stiffness_target_);

  // Initialize variables for tool compensation from yaml config file
  activate_tool_compensation_ = true;
  tool_compensation_force_.setZero();
  std::vector<double> external_tool_compensation;
  // tool_compensation_force_ << 0.46, -0.17, -1.64, 0, 0, 0;  //read from yaml
  if (!node_handle.getParam("external_tool_compensation", external_tool_compensation) || external_tool_compensation.size() != 6) {
      ROS_ERROR(
          "CartesianPoseImpedanceController: Invalid or no external_tool_compensation parameters provided, "
          "aborting controller init!");
      return false;
    }
  for (size_t i = 0; i < 6; ++i) 
    tool_compensation_force_[i] = external_tool_compensation.at(i);
  //ROS_INFO_STREAM("External tool compensation force: " << std::endl << tool_compensation_force_);

  // Initialize variables for nullspace control from yaml config file
  q_d_nullspace_.setZero();
  std::vector<double> q_nullspace;
  if (node_handle.getParam("q_nullspace", q_nullspace)) {
    q_d_nullspace_initialized_ = true;
    if (q_nullspace.size() != 7) {
      ROS_ERROR(
        "CartesianPoseImpedanceController: Invalid or no q_nullspace parameters provided, "
        "aborting controller init!");
      return false;
    }
    for (size_t i = 0; i < 7; ++i) 
      q_d_nullspace_[i] = q_nullspace.at(i);
    // ROS_INFO_STREAM("Desired nullspace position (from YAML): " << std::endl << q_d_nullspace_);
  }
  return true;
}

void CartesianPoseImpedanceController::starting(const ros::Time& /*time*/) {

  // Get robot current/initial joint state
  franka::RobotState initial_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  
  _Gravity<< 0.0, 0.0, -2.9;
  _update_counter = 0;
  _action_counter = -100;
  _episode_counter = 0;
  _print_flag = true;
  std::string action_csv = "/home/yzc/project/robotswitch/src/franka_interactive_controllers/doc/actions.csv";
  action_file = std::ofstream(action_csv, std::ios::out);
  action_file << "action" << std::endl;

  std::string observation_csv = "/home/yzc/project/robotswitch/src/franka_interactive_controllers/doc/observations.csv";
  state_file = std::ofstream(observation_csv, std::ios::out);
  state_file << "observation" << std::endl;

  std::string episode_end_csv = "/home/yzc/project/robotswitch/src/franka_interactive_controllers/doc/episode_ends.csv";
  episode_end_file = std::ofstream(episode_end_csv, std::ios::out);
  episode_end_file << "episode_ends" << std::endl;
  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
std::cout << "Initial Transform:\n" << initial_transform.matrix() << std::endl;

  // set desired point to current state
  position_d_           = initial_transform.translation();
  orientation_d_        = Eigen::Quaterniond(initial_transform.linear());

  position_init_        = initial_transform.translation();
  orientation_init_     = Eigen::Quaterniond(initial_transform.linear());
  position_d_target_    = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());

  if (!q_d_nullspace_initialized_) {
    q_d_nullspace_ = q_initial;
    q_d_nullspace_initialized_ = true;
    //ROS_INFO_STREAM("Desired nullspace position (from q_initial): " << std::endl << q_d_nullspace_);
  }
}

void CartesianPoseImpedanceController::update(const ros::Time& /*time*/,
                                                 const ros::Duration& /*period*/) {
  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 7> gravity_array = model_handle_->getGravity();

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());

if (position(2) < 0.18)
{
  if(_print_flag)
  {
      if (_action_counter >= 0 && _action_counter % 100 == 0)
      {
        action_file << "\"[";
        action_file << position(0) - position_init_(0)<< ", "
                     << position(1) - position_init_(1)<< ", "
                     << position(2) - position_init_(2)<< ", "
                     << orientation.w() << ", "
                     << orientation.x() << ", "
                     << orientation.y() << ", "
                     << orientation.z() << "]\"" << std::endl;
      }
      _action_counter += 1;
  }

  // episode end record
  if (position(2) < 0.075){// cutting to end threshold = 0.07m
    episode_end_file << _episode_counter << std::endl;
    _print_flag = false;
  }

  if(_print_flag)
  {
      // 10Hz记录一次
      if (_update_counter % 100 == 0)
      {
        // state record
          Eigen::Matrix3d rotation_matrix = transform.rotation();
          Eigen::Vector3d GinF = rotation_matrix.transpose() * _Gravity;
          Eigen::Vector3d compensated_force(
              robot_state.K_F_ext_hat_K[0] + GinF[0],
              robot_state.K_F_ext_hat_K[1] + GinF[1],
              robot_state.K_F_ext_hat_K[2] + GinF[2]);
          force_in_world = rotation_matrix * compensated_force; // compensate force 
          state_file << "\"[";
          state_file << position(0) - position_init_(0)<< ", "
                     << position(1) - position_init_(1)<< ", "
                     << position(2) - position_init_(2)<< ", "
                     << orientation.w() << ", "
                     << orientation.x() << ", "
                     << orientation.y() << ", "
                     << orientation.z() << ", "
                     << force_in_world[0] << ", " 
                     << force_in_world[1] << ", " 
                     << force_in_world[2] << "]\"";
          state_file << std::endl;
      }
      _update_counter += 1;
      _episode_counter += 1;
  }
}
    // obs_array.data.clear();
    // obs_array.data.push_back(position(0));
    // obs_array.data.push_back(position(1));
    // obs_array.data.push_back(position(2));
    // obs_array.data.push_back(orientation.w());
    // obs_array.data.push_back(orientation.x());
    // obs_array.data.push_back(orientation.y());
    // obs_array.data.push_back(orientation.z());
    // obs_array.data.push_back(force_in_world[0]);
    // obs_array.data.push_back(force_in_world[1]);
    // obs_array.data.push_back(force_in_world[2]);
    // pub_observation.publish(obs_array);
  //////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////              COMPUTING TASK CONTROL TORQUE           //////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////

  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7), tau_tool(7);


  // ROS_INFO_STREAM ("Doing Cartesian Impedance Control");            
  // compute error to desired pose
  // position error
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - position_d_;

  // orientation error
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  error.tail(3) << -transform.linear() * error.tail(3);

  // Cartesian PD control with damping ratio = 1
  Eigen::Matrix<double, 6, 1> velocity;
  velocity << jacobian * dq;
  Eigen::VectorXd     F_ee_des_;
  F_ee_des_.resize(6);
  F_ee_des_ << -cartesian_stiffness_ * error - cartesian_damping_ * velocity;
  tau_task << jacobian.transpose() * F_ee_des_;
  // ROS_WARN_STREAM_THROTTLE(0.5, "Current Velocity Norm:" << velocity.head(3).norm());
  // ROS_WARN_STREAM_THROTTLE(0.5, "Classic Linear Control Force:" << F_ee_des_.head(3).norm());
  // ROS_WARN_STREAM_THROTTLE(0.5, "Classic Angular Control Force :" << F_ee_des_.tail(3).norm());

  //////////////////////////////////////////////////////////////////////////////////////////////////


  // pseudoinverse for nullspace handling
  // kinematic pseudoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  // nullspace PD control with damping ratio = 1
  // ROS_WARN_STREAM_THROTTLE(0.5, "Nullspace stiffness:" << nullspace_stiffness_); 
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                        (2.0 * sqrt(nullspace_stiffness_)) * dq);

  // ROS_WARN_STREAM_THROTTLE(0.5, "Nullspace torques:" << tau_nullspace.transpose());    
  // double tau_nullspace_0 = tau_nullspace(0);
  // tau_nullspace.setZero();
  // tau_nullspace[0] = tau_nullspace_0; 

  // Compute tool compensation (scoop/camera in scooping task)
  if (activate_tool_compensation_)
    tau_tool << jacobian.transpose() * tool_compensation_force_;
  else
    tau_tool.setZero();

  // Desired torque
  tau_d << tau_task + tau_nullspace + coriolis - tau_tool;
  // ROS_WARN_STREAM_THROTTLE(0.5, "Desired control torque:" << tau_d.transpose());

  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////

  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  cartesian_stiffness_  = cartesian_stiffness_target_;
  cartesian_damping_    = cartesian_damping_target_;
  nullspace_stiffness_  = nullspace_stiffness_target_;
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
}

Eigen::Matrix<double, 7, 1> CartesianPoseImpedanceController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

void CartesianPoseImpedanceController::complianceParamCallback(
    franka_interactive_controllers::minimal_compliance_paramConfig& config,
    uint32_t /*level*/) {

  activate_tool_compensation_ = config.activate_tool_compensation;
}

void CartesianPoseImpedanceController::desiredPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  //pose sequence cannot change！
  // ROS_INFO_STREAM("[CALLBACK] Desired ee position from DS: " << position_d_target_);
  if (msg->pose.position.y > 0.2 || msg->pose.position.y < -0.2)
  {
  }
  else if (msg->pose.position.z > 0.25 || msg->pose.position.z < -0.25)
  {
  }else if(msg->pose.position.x > 0.2 || msg->pose.position.x < -0.2)
  {
  }else{
      position_d_target_ << position_init_(0) + msg->pose.position.x, position_init_(1) + msg->pose.position.y, position_init_(2) + msg->pose.position.z;
    }
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);

  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
}

}  // namespace franka_interactive_controllers

PLUGINLIB_EXPORT_CLASS(franka_interactive_controllers::CartesianPoseImpedanceController,
                       controller_interface::ControllerBase)
