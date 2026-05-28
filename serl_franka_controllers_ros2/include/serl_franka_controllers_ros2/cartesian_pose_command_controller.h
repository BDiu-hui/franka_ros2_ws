// Refered to https://github.com/frankarobotics/franka_ros2

#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Dense>
#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <franka_semantic_components/franka_cartesian_pose_interface.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace serl_franka_controllers {

class CartesianPoseCommandController : public controller_interface::ControllerInterface {
 public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  void target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  std::unique_ptr<franka_semantic_components::FrankaCartesianPoseInterface> franka_cartesian_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_subscriber_;

  std::mutex target_mutex_;
  Eigen::Quaterniond current_orientation_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d current_position_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond target_orientation_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d target_position_{Eigen::Vector3d::Zero()};

  bool have_target_{false};
  bool initialization_flag_{true};
  std::string robot_type_{"fr3"};
  std::string arm_prefix_;
  double max_linear_velocity_{0.02};
  double max_angular_velocity_{0.2};
  const bool k_elbow_activated_{false};
};

}  // namespace serl_franka_controllers
