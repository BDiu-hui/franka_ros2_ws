// Refered to https://github.com/frankarobotics/franka_ros2

#pragma once

#include <array>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace serl_franka_controllers {

class JointPositionController : public controller_interface::ControllerInterface {
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

 private:
  bool read_target_joint_positions();

  std::vector<std::string> joint_names_;
  std::array<double, 7> initial_pose_{};
  std::array<double, 7> reset_pose_{};
  double trajectory_duration_{20.0};
  double elapsed_time_{0.0};
};

}  // namespace serl_franka_controllers
