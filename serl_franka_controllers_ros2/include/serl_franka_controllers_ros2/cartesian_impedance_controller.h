// Refered to https://github.com/frankarobotics/franka_ros2

#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <type_traits>
#include <vector>

#include <Eigen/Dense>
#include <controller_interface/controller_interface.hpp>
#include <franka/robot_state.h>
#include <franka_semantic_components/franka_robot_model.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_lifecycle/state.hpp>
// Humble still needs this define for the non-polling publisher implementation.
#define NON_POLLING 1  // NOLINT
#include <realtime_tools/realtime_publisher.hpp>

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
  struct ComplianceParams {
    double translational_stiffness{2000.0};
    double translational_damping{89.0};
    double rotational_stiffness{300.0}; // 150.0
    double rotational_damping{7.0};
    double nullspace_stiffness{0.2};
    double joint1_nullspace_stiffness{100.0};
    double translational_clip_neg_x{0.01};
    double translational_clip_neg_y{0.01};
    double translational_clip_neg_z{0.01};
    double translational_clip_x{0.01};
    double translational_clip_y{0.01};
    double translational_clip_z{0.01};
    double rotational_clip_neg_x{0.05};
    double rotational_clip_neg_y{0.05};
    double rotational_clip_neg_z{0.05};
    double rotational_clip_x{0.05};
    double rotational_clip_y{0.05};
    double rotational_clip_z{0.05};
    double translational_ki{0.0};
    double rotational_ki{0.0};
    double filter_params{0.005};
    double elbow_stiffness{0.0};
    double elbow_damping{0.0};
  };

  Eigen::Matrix<double, 7, 1> saturate_torque_rate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_j_d) const;
  void equilibrium_pose_callback(
      const serl_franka_controllers_ros2::msg::CartesianImpedanceCommand::SharedPtr msg);
  Eigen::Vector3d compute_elbow_position(const Eigen::Matrix<double, 7, 1>& q) const;
  void apply_compliance_params(const ComplianceParams& params);
  bool read_parameters();
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
  static constexpr double kDeltaTauMax = 1.0;

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

  ComplianceParams compliance_params_;
  std::mutex target_mutex_;

  Eigen::Matrix<double, 6, 6> cartesian_stiffness_{Eigen::Matrix<double, 6, 6>::Zero()};
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_{Eigen::Matrix<double, 6, 6>::Zero()};
  Eigen::Matrix<double, 6, 6> cartesian_damping_{Eigen::Matrix<double, 6, 6>::Zero()};
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_{Eigen::Matrix<double, 6, 6>::Zero()};
  Eigen::Matrix<double, 6, 6> ki_{Eigen::Matrix<double, 6, 6>::Zero()};
  Eigen::Matrix<double, 6, 6> ki_target_{Eigen::Matrix<double, 6, 6>::Zero()};

  Eigen::Matrix<double, 3, 1> translational_clip_min_{Eigen::Matrix<double, 3, 1>::Zero()};
  Eigen::Matrix<double, 3, 1> translational_clip_max_{Eigen::Matrix<double, 3, 1>::Zero()};
  Eigen::Matrix<double, 3, 1> rotational_clip_min_{Eigen::Matrix<double, 3, 1>::Zero()};
  Eigen::Matrix<double, 3, 1> rotational_clip_max_{Eigen::Matrix<double, 3, 1>::Zero()};
  Eigen::Matrix<double, 7, 1> q_d_nullspace_{Eigen::Matrix<double, 7, 1>::Zero()};
  Eigen::Matrix<double, 7, 1> q_master_target_{Eigen::Matrix<double, 7, 1>::Zero()};
  Eigen::Matrix<double, 7, 1> q_master_{Eigen::Matrix<double, 7, 1>::Zero()};
  bool have_master_q_{false};
  Eigen::Matrix<double, 6, 1> error_{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix<double, 6, 1> error_i_{Eigen::Matrix<double, 6, 1>::Zero()};

  Eigen::Vector3d position_d_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation_d_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d position_d_target_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation_d_target_{Eigen::Quaterniond::Identity()};

  double nullspace_stiffness_{20.0};
  double nullspace_stiffness_target_{20.0};
  double joint1_nullspace_stiffness_{20.0};
  double joint1_nullspace_stiffness_target_{20.0};
  double elbow_stiffness_{0.0};
  double elbow_stiffness_target_{0.0};
  double elbow_damping_{0.0};
  double elbow_damping_target_{0.0};
};

}  // namespace serl_franka_controllers
