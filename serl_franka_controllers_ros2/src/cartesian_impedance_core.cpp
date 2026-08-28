#include <serl_franka_controllers_ros2/cartesian_impedance_core.hpp>

#include <algorithm>
#include <array>
#include <cmath>

namespace serl_franka_controllers {

void CartesianImpedanceCore::reset(const ImpedanceInput& input,
                                   const ImpedanceTarget& target,
                                   const ComplianceParams& params) {
  q_d_nullspace_ = input.q;
  q_master_ = target.has_master_q ? target.master_q : input.q;
  position_d_ = input.ee_pose.translation();
  orientation_d_ = Eigen::Quaterniond(input.ee_pose.linear());
  error_i_.setZero();
  last_target_sequence_ = target.sequence;
  applyParamTargets(params, true);
}

bool CartesianImpedanceCore::needsElbowInput(const ImpedanceTarget& target,
                                             const ComplianceParams& params) const {
  return target.has_master_q && (elbow_stiffness_ > 0.0 || params.elbow_stiffness > 0.0);
}

Vector7d CartesianImpedanceCore::update(const ImpedanceInput& input,
                                        const ImpedanceTarget& target,
                                        const ComplianceParams& params,
                                        const std::optional<ElbowInput>& elbow) {
  applyParamTargets(params, false);
  if (target.sequence != last_target_sequence_) {
    error_i_.setZero();
    last_target_sequence_ = target.sequence;
  }

  Eigen::Matrix<double, 6, 1> error;
  error.head(3) = input.ee_pose.translation() - position_d_;
  error.head(3) = error.head(3).cwiseMax(translational_clip_min_).cwiseMin(translational_clip_max_);

  Eigen::Quaterniond orientation(input.ee_pose.linear());
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() = -orientation.coeffs();
  }
  const Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  error.tail(3) = -input.ee_pose.linear() * error.tail(3);
  error.tail(3) = error.tail(3).cwiseMax(rotational_clip_min_).cwiseMin(rotational_clip_max_);

  error_i_.head(3) = (error_i_.head(3) + error.head(3)).cwiseMax(-0.1).cwiseMin(0.1);
  error_i_.tail(3) = (error_i_.tail(3) + error.tail(3)).cwiseMax(-0.3).cwiseMin(0.3);

  const Eigen::Matrix<double, 6, 6> jj_t_damped =
      input.jacobian * input.jacobian.transpose() +
      (kDampedPseudoInverseLambda * kDampedPseudoInverseLambda) *
          Eigen::Matrix<double, 6, 6>::Identity();
  const Matrix67d jacobian_transpose_pinv = jj_t_damped.ldlt().solve(input.jacobian);
  const Eigen::Matrix<double, 7, 7> nullspace_projector =
      Eigen::Matrix<double, 7, 7>::Identity() -
      input.jacobian.transpose() * jacobian_transpose_pinv;

  const Vector7d tau_task =
      input.jacobian.transpose() *
      (-stiffness_ * error - damping_ * (input.jacobian * input.dq) - ki_ * error_i_);

  Vector7d qe = q_d_nullspace_ - input.q;
  qe.head(1) *= joint1_nullspace_stiffness_;
  Vector7d dqe = input.dq;
  dqe.head(1) *= 2.0 * std::sqrt(joint1_nullspace_stiffness_);
  const Vector7d tau_nullspace =
      nullspace_projector *
      (nullspace_stiffness_ * qe - (2.0 * std::sqrt(nullspace_stiffness_)) * dqe);

  Vector7d tau_elbow_null = Vector7d::Zero();
  if (elbow.has_value() && target.has_master_q && elbow_stiffness_ > 0.0) {
    const Eigen::Vector3d error_position = computeElbowPosition(q_master_) - elbow->position;
    const Vector7d tau_elbow =
        elbow->jacobian.transpose() *
        (elbow_stiffness_ * error_position - elbow_damping_ * (elbow->jacobian * input.dq));
    tau_elbow_null = nullspace_projector * tau_elbow;
  }

  const Vector7d torque =
      saturateTorqueRate(tau_task + tau_nullspace + tau_elbow_null + input.coriolis, input.tau_j_d);

  const double filter = std::clamp(params.filter_params, 0.0, 1.0);
  stiffness_ = filter * stiffness_target_ + (1.0 - filter) * stiffness_;
  damping_ = filter * damping_target_ + (1.0 - filter) * damping_;
  ki_ = filter * ki_target_ + (1.0 - filter) * ki_;
  nullspace_stiffness_ =
      filter * nullspace_stiffness_target_ + (1.0 - filter) * nullspace_stiffness_;
  joint1_nullspace_stiffness_ =
      filter * joint1_nullspace_stiffness_target_ + (1.0 - filter) * joint1_nullspace_stiffness_;
  elbow_stiffness_ = filter * elbow_stiffness_target_ + (1.0 - filter) * elbow_stiffness_;
  elbow_damping_ = filter * elbow_damping_target_ + (1.0 - filter) * elbow_damping_;
  position_d_ = filter * target.position + (1.0 - filter) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter, target.orientation);
  q_master_ = filter * target.master_q + (1.0 - filter) * q_master_;

  return torque;
}

void CartesianImpedanceCore::applyParamTargets(const ComplianceParams& params, bool initialize) {
  stiffness_target_.setIdentity();
  stiffness_target_.topLeftCorner(3, 3) =
      params.translational_stiffness * Eigen::Matrix3d::Identity();
  stiffness_target_.bottomRightCorner(3, 3) =
      params.rotational_stiffness * Eigen::Matrix3d::Identity();
  damping_target_.setIdentity();
  damping_target_.topLeftCorner(3, 3) = params.translational_damping * Eigen::Matrix3d::Identity();
  damping_target_.bottomRightCorner(3, 3) = params.rotational_damping * Eigen::Matrix3d::Identity();
  ki_target_.setIdentity();
  ki_target_.topLeftCorner(3, 3) = params.translational_ki * Eigen::Matrix3d::Identity();
  ki_target_.bottomRightCorner(3, 3) = params.rotational_ki * Eigen::Matrix3d::Identity();
  nullspace_stiffness_target_ = std::max(0.0, params.nullspace_stiffness);
  joint1_nullspace_stiffness_target_ = std::max(0.0, params.joint1_nullspace_stiffness);
  elbow_stiffness_target_ = std::max(0.0, params.elbow_stiffness);
  elbow_damping_target_ = std::max(0.0, params.elbow_damping);
  translational_clip_min_ << -params.translational_clip_neg_x, -params.translational_clip_neg_y,
      -params.translational_clip_neg_z;
  translational_clip_max_ << params.translational_clip_x, params.translational_clip_y,
      params.translational_clip_z;
  rotational_clip_min_ << -params.rotational_clip_neg_x, -params.rotational_clip_neg_y,
      -params.rotational_clip_neg_z;
  rotational_clip_max_ << params.rotational_clip_x, params.rotational_clip_y,
      params.rotational_clip_z;

  if (initialize) {
    stiffness_ = stiffness_target_;
    damping_ = damping_target_;
    ki_ = ki_target_;
    nullspace_stiffness_ = nullspace_stiffness_target_;
    joint1_nullspace_stiffness_ = joint1_nullspace_stiffness_target_;
    elbow_stiffness_ = elbow_stiffness_target_;
    elbow_damping_ = elbow_damping_target_;
  }
}

Vector7d CartesianImpedanceCore::saturateTorqueRate(const Vector7d& calculated,
                                                    const Vector7d& previous) const {
  Vector7d saturated;
  for (Eigen::Index index = 0; index < saturated.size(); ++index) {
    saturated(index) = previous(index) +
                       std::clamp(calculated(index) - previous(index), -kDeltaTauMax, kDeltaTauMax);
  }
  return saturated;
}

Eigen::Vector3d CartesianImpedanceCore::computeElbowPosition(const Vector7d& q) const {
  static constexpr std::array<double, 4> kAPrevious = {0.0, 0.0, 0.0, 0.0825};
  static constexpr std::array<double, 4> kAlphaPrevious = {0.0, -M_PI_2, M_PI_2, M_PI_2};
  static constexpr std::array<double, 4> kD = {0.333, 0.0, 0.316, 0.0};

  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  for (int index = 0; index < 4; ++index) {
    const double ca = std::cos(kAlphaPrevious[index]);
    const double sa = std::sin(kAlphaPrevious[index]);
    const double ct = std::cos(q(index));
    const double st = std::sin(q(index));
    Eigen::Matrix4d link;
    link << ct, -st, 0.0, kAPrevious[index], st * ca, ct * ca, -sa, -sa * kD[index], st * sa,
        ct * sa, ca, ca * kD[index], 0.0, 0.0, 0.0, 1.0;
    transform *= link;
  }
  return transform.block<3, 1>(0, 3);
}

}  // namespace serl_franka_controllers
