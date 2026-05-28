/*
Refered to source file:
  https://github.com/frankarobotics/franka_ros2
*/

#include <serl_franka_controllers_ros2/joint_position_controller.h>

#include <algorithm>
#include <exception>

#include <pluginlib/class_list_macros.hpp>

namespace serl_franka_controllers {

controller_interface::InterfaceConfiguration
JointPositionController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint_name : joint_names_) {
    config.names.push_back(joint_name + "/position");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointPositionController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint_name : joint_names_) {
    config.names.push_back(joint_name + "/position");
  }
  return config;
}

CallbackReturn JointPositionController::on_init() {
  try {
    auto_declare<std::vector<std::string>>("joint_names",
                                           {"panda_joint1", "panda_joint2", "panda_joint3",
                                            "panda_joint4", "panda_joint5", "panda_joint6",
                                            "panda_joint7"});
    auto_declare<std::vector<double>>("target_joint_positions",
                                      {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785});
    auto_declare<double>("trajectory_duration", 20.0);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Init failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointPositionController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  joint_names_ = get_node()->get_parameter("joint_names").as_string_array();

  if (joint_names_.size() != 7) {
    RCLCPP_ERROR(get_node()->get_logger(), "joint_names must contain exactly 7 entries");
    return CallbackReturn::ERROR;
  }
  if (!read_target_joint_positions()) {
    return CallbackReturn::ERROR;
  }
  trajectory_duration_ = get_node()->get_parameter("trajectory_duration").as_double();
  if (trajectory_duration_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "trajectory_duration must be > 0.0");
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointPositionController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (!read_target_joint_positions()) {
    return CallbackReturn::ERROR;
  }
  for (size_t i = 0; i < 7; ++i) {
    initial_pose_[i] = state_interfaces_[i].get_value();
    command_interfaces_[i].set_value(initial_pose_[i]);
  }
  elapsed_time_ = 0.0;
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type JointPositionController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  elapsed_time_ += period.seconds();

  for (size_t i = 0; i < 7; ++i) {
    if (elapsed_time_ > trajectory_duration_) {
      command_interfaces_[i].set_value(reset_pose_[i]);
    } else {
      const double t = std::clamp(elapsed_time_ / trajectory_duration_, 0.0, 1.0);
      const double alpha = t * t * t * (10.0 + t * (-15.0 + 6.0 * t));
      command_interfaces_[i].set_value(alpha * reset_pose_[i] + (1.0 - alpha) * initial_pose_[i]);
    }
  }

  return controller_interface::return_type::OK;
}

bool JointPositionController::read_target_joint_positions() {
  const auto target_positions = get_node()->get_parameter("target_joint_positions").as_double_array();
  if (target_positions.size() != 7) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "target_joint_positions must contain exactly 7 entries");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    reset_pose_[i] = target_positions[i];
  }
  return true;
}

}  // namespace serl_franka_controllers

PLUGINLIB_EXPORT_CLASS(serl_franka_controllers::JointPositionController,
                       controller_interface::ControllerInterface)
