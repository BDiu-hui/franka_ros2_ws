// Refered to https://github.com/frankarobotics/franka_ros2

#include <serl_franka_controllers_ros2/cartesian_pose_command_controller.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <string>

#include <pluginlib/class_list_macros.hpp>

namespace serl_franka_controllers {

controller_interface::InterfaceConfiguration
CartesianPoseCommandController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_pose_->get_command_interface_names();
  return config;
}

controller_interface::InterfaceConfiguration
CartesianPoseCommandController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_pose_->get_state_interface_names();
  config.names.push_back(arm_prefix_ + robot_type_ + "/robot_time");
  return config;
}

CallbackReturn CartesianPoseCommandController::on_init() {
  try {
    auto_declare<std::string>("robot_type", "fr3");
    auto_declare<std::string>("arm_prefix", "");
    auto_declare<double>("max_linear_velocity", 0.02);
    auto_declare<double>("max_angular_velocity", 0.2);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Init failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianPoseCommandController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  robot_type_ = get_node()->get_parameter("robot_type").as_string();
  arm_prefix_ = get_node()->get_parameter("arm_prefix").as_string();
  arm_prefix_ = arm_prefix_.empty() ? "" : arm_prefix_ + "_";
  max_linear_velocity_ = get_node()->get_parameter("max_linear_velocity").as_double();
  max_angular_velocity_ = get_node()->get_parameter("max_angular_velocity").as_double();

  franka_cartesian_pose_ =
      std::make_unique<franka_semantic_components::FrankaCartesianPoseInterface>(
          franka_semantic_components::FrankaCartesianPoseInterface(arm_prefix_,
                                                                   k_elbow_activated_));

  target_pose_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/target_pose", rclcpp::SystemDefaultsQoS(),
      std::bind(&CartesianPoseCommandController::target_pose_callback, this, std::placeholders::_1));

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianPoseCommandController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_pose_->assign_loaned_command_interfaces(command_interfaces_);
  franka_cartesian_pose_->assign_loaned_state_interfaces(state_interfaces_);

  initialization_flag_ = true;
  auto current_pose = franka_cartesian_pose_->getCurrentOrientationAndTranslation();
  auto current_pose_matrix = franka_cartesian_pose_->getCurrentPoseMatrix();

  if (!franka_cartesian_pose_->setCommand(current_pose_matrix)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Failed to seed cartesian pose command interfaces with the current pose on activate");
    return CallbackReturn::ERROR;
  }

  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    current_orientation_ = std::get<0>(current_pose);
    current_position_ = std::get<1>(current_pose);
    target_orientation_ = std::get<0>(current_pose);
    target_position_ = std::get<1>(current_pose);
    have_target_ = true;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianPoseCommandController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_pose_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianPoseCommandController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  if (initialization_flag_) {
    auto current_pose = franka_cartesian_pose_->getCurrentOrientationAndTranslation();
    {
      std::lock_guard<std::mutex> lock(target_mutex_);
      current_orientation_ = std::get<0>(current_pose);
      current_position_ = std::get<1>(current_pose);
      target_orientation_ = std::get<0>(current_pose);
      target_position_ = std::get<1>(current_pose);
      have_target_ = true;
    }
    initialization_flag_ = false;
  }

  const double dt = std::max(period.seconds(), 1e-6);
  Eigen::Quaterniond target_orientation;
  Eigen::Quaterniond current_orientation;
  Eigen::Vector3d target_position;
  Eigen::Vector3d current_position;
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    if (!have_target_) {
      return controller_interface::return_type::OK;
    }
    current_orientation = current_orientation_;
    current_position = current_position_;
    target_orientation = target_orientation_;
    target_position = target_position_;
  }

  const double max_linear_step = max_linear_velocity_ * dt;
  Eigen::Vector3d position_error = target_position - current_position;
  const double position_error_norm = position_error.norm();
  if (position_error_norm > max_linear_step && max_linear_step > 0.0) {
    current_position += position_error * (max_linear_step / position_error_norm);
  } else {
    current_position = target_position;
  }

  const double max_angular_step = max_angular_velocity_ * dt;
  double angular_distance = current_orientation.angularDistance(target_orientation);
  if (angular_distance > max_angular_step && max_angular_step > 0.0) {
    const double interpolation = std::clamp(max_angular_step / angular_distance, 0.0, 1.0);
    current_orientation = current_orientation.slerp(interpolation, target_orientation);
  } else {
    current_orientation = target_orientation;
  }
  current_orientation.normalize();

  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    current_orientation_ = current_orientation;
    current_position_ = current_position;
  }

  if (franka_cartesian_pose_->setCommand(current_orientation, current_position)) {
    return controller_interface::return_type::OK;
  }

  RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                        "Failed to send cartesian pose command");
  return controller_interface::return_type::ERROR;
}

void CartesianPoseCommandController::target_pose_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(target_mutex_);
  target_position_ =
      Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  target_orientation_ = Eigen::Quaterniond(
      msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z);
  target_orientation_.normalize();
  have_target_ = true;
}

}  // namespace serl_franka_controllers

PLUGINLIB_EXPORT_CLASS(serl_franka_controllers::CartesianPoseCommandController,
                       controller_interface::ControllerInterface)
