// Refered to https://github.com/frankarobotics/franka_ros2

#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include <franka/robot_state.h>
#include <Eigen/Dense>
#include <controller_interface/controller_interface.hpp>
#include <franka_semantic_components/franka_robot_model.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_lifecycle/state.hpp>
// Humble still needs this define for the non-polling publisher implementation.
#define NON_POLLING 1  // NOLINT
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>

#include <serl_franka_controllers_ros2/cartesian_impedance_core.hpp>
#include <serl_franka_controllers_ros2/msg/cartesian_impedance_command.hpp>
#include <serl_franka_controllers_ros2/msg/zero_jacobian.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace serl_franka_controllers {

class CartesianImpedanceController : public controller_interface::ControllerInterface {
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
  void equilibrium_pose_callback(
      const serl_franka_controllers_ros2::msg::CartesianImpedanceCommand::SharedPtr msg);
  bool read_parameters();
  void set_jacobian_publish_rate(double publish_rate_hz);
  rcl_interfaces::msg::SetParametersResult on_parameters_set(
      const std::vector<rclcpp::Parameter>& parameters);
  franka::RobotState* get_robot_state_ptr();
  void publish_zero_jacobian();

  std::vector<std::string> joint_names_;
  std::string arm_id_{"panda"};
  std::string robot_type_;
  std::string arm_prefix_;
  std::string hardware_prefix_;

  static constexpr size_t kNumJoints = 7;
  std::array<double, 42> jacobian_array_{};
  franka::RobotState* robot_state_ptr_{nullptr};
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;

  std::shared_ptr<rclcpp::Publisher<serl_franka_controllers_ros2::msg::ZeroJacobian>>
      jacobian_publisher_;
  std::shared_ptr<
      realtime_tools::RealtimePublisher<serl_franka_controllers_ros2::msg::ZeroJacobian>>
      realtime_jacobian_publisher_;
  rclcpp::Subscription<serl_franka_controllers_ros2::msg::CartesianImpedanceCommand>::SharedPtr
      equilibrium_pose_subscriber_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  int jacobian_publish_decimation_{10};
  int jacobian_publish_counter_{0};

  ComplianceParams compliance_params_;
  realtime_tools::RealtimeBuffer<ComplianceParams> compliance_params_buffer_;
  realtime_tools::RealtimeBuffer<ImpedanceTarget> target_command_buffer_;
  CartesianImpedanceCore impedance_core_;
};

}  // namespace serl_franka_controllers
