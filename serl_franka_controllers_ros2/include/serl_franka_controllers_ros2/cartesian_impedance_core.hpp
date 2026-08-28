#pragma once

#include <cstdint>
#include <optional>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace serl_franka_controllers {

using Vector7d = Eigen::Matrix<double, 7, 1>;
using Matrix67d = Eigen::Matrix<double, 6, 7>;
using Matrix37d = Eigen::Matrix<double, 3, 7>;

struct ComplianceParams {
  double translational_stiffness{2020.0};
  double translational_damping{89.0};
  double rotational_stiffness{300.0};
  double rotational_damping{7.0};
  double nullspace_stiffness{0.5};
  double joint1_nullspace_stiffness{100.0};
  double translational_clip_neg_x{0.03};
  double translational_clip_neg_y{0.03};
  double translational_clip_neg_z{0.03};
  double translational_clip_x{0.03};
  double translational_clip_y{0.03};
  double translational_clip_z{0.03};
  double rotational_clip_neg_x{0.05};
  double rotational_clip_neg_y{0.05};
  double rotational_clip_neg_z{0.05};
  double rotational_clip_x{0.05};
  double rotational_clip_y{0.05};
  double rotational_clip_z{0.05};
  double translational_ki{0.0};
  double rotational_ki{0.0};
  double filter_params{0.02};
  double elbow_stiffness{0.0};
  double elbow_damping{0.0};
};

struct ImpedanceTarget {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  Vector7d master_q{Vector7d::Zero()};
  bool has_master_q{false};
  std::uint64_t sequence{0};
};

struct ImpedanceInput {
  Vector7d q{Vector7d::Zero()};
  Vector7d dq{Vector7d::Zero()};
  Vector7d tau_j_d{Vector7d::Zero()};
  Vector7d coriolis{Vector7d::Zero()};
  Matrix67d jacobian{Matrix67d::Zero()};
  Eigen::Isometry3d ee_pose{Eigen::Isometry3d::Identity()};
};

struct ElbowInput {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Matrix37d jacobian{Matrix37d::Zero()};
};

class CartesianImpedanceCore {
 public:
  void reset(const ImpedanceInput& input,
             const ImpedanceTarget& target,
             const ComplianceParams& params);
  [[nodiscard]] bool needsElbowInput(const ImpedanceTarget& target,
                                     const ComplianceParams& params) const;
  [[nodiscard]] Vector7d update(const ImpedanceInput& input,
                                const ImpedanceTarget& target,
                                const ComplianceParams& params,
                                const std::optional<ElbowInput>& elbow = std::nullopt);

 private:
  static constexpr double kDeltaTauMax = 1.0;
  static constexpr double kDampedPseudoInverseLambda = 0.2;

  void applyParamTargets(const ComplianceParams& params, bool initialize);
  [[nodiscard]] Vector7d saturateTorqueRate(const Vector7d& calculated,
                                            const Vector7d& previous) const;
  [[nodiscard]] Eigen::Vector3d computeElbowPosition(const Vector7d& q) const;

  Eigen::Matrix<double, 6, 6> stiffness_{Eigen::Matrix<double, 6, 6>::Zero()};
  Eigen::Matrix<double, 6, 6> stiffness_target_{Eigen::Matrix<double, 6, 6>::Zero()};
  Eigen::Matrix<double, 6, 6> damping_{Eigen::Matrix<double, 6, 6>::Zero()};
  Eigen::Matrix<double, 6, 6> damping_target_{Eigen::Matrix<double, 6, 6>::Zero()};
  Eigen::Matrix<double, 6, 6> ki_{Eigen::Matrix<double, 6, 6>::Zero()};
  Eigen::Matrix<double, 6, 6> ki_target_{Eigen::Matrix<double, 6, 6>::Zero()};
  Eigen::Vector3d translational_clip_min_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d translational_clip_max_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d rotational_clip_min_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d rotational_clip_max_{Eigen::Vector3d::Zero()};
  Vector7d q_d_nullspace_{Vector7d::Zero()};
  Vector7d q_master_{Vector7d::Zero()};
  Eigen::Matrix<double, 6, 1> error_i_{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Vector3d position_d_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation_d_{Eigen::Quaterniond::Identity()};
  double nullspace_stiffness_{0.0};
  double nullspace_stiffness_target_{0.0};
  double joint1_nullspace_stiffness_{0.0};
  double joint1_nullspace_stiffness_target_{0.0};
  double elbow_stiffness_{0.0};
  double elbow_stiffness_target_{0.0};
  double elbow_damping_{0.0};
  double elbow_damping_target_{0.0};
  std::uint64_t last_target_sequence_{0};
};

}  // namespace serl_franka_controllers
