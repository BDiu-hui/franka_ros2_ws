#include <cmath>

#include <serl_franka_controllers_ros2/cartesian_impedance_core.hpp>

int main() {
  using namespace serl_franka_controllers;

  ComplianceParams params;
  params.translational_stiffness = 200.0;
  params.translational_damping = 0.0;
  params.rotational_stiffness = 0.0;
  params.rotational_damping = 0.0;
  params.nullspace_stiffness = 0.0;
  params.joint1_nullspace_stiffness = 0.0;
  params.filter_params = 1.0;

  ImpedanceInput input;
  input.jacobian(0, 0) = 1.0;
  input.coriolis.setConstant(0.25);
  input.tau_j_d = input.coriolis;
  ImpedanceTarget target;

  CartesianImpedanceCore core;
  core.reset(input, target, params);
  const Vector7d equilibrium_torque = core.update(input, target, params);
  if ((equilibrium_torque - input.coriolis).norm() >= 1e-12) {
    return 1;
  }

  input.coriolis.setZero();
  input.tau_j_d.setZero();
  target.position.x() = 0.01;
  ++target.sequence;
  (void)core.update(input, target, params);
  const Vector7d displaced_torque = core.update(input, target, params);
  if (displaced_torque(0) <= 0.0 || displaced_torque.cwiseAbs().maxCoeff() > 1.0 + 1e-12) {
    return 1;
  }

  ComplianceParams rotation_params = params;
  rotation_params.translational_stiffness = 0.0;
  rotation_params.rotational_stiffness = 200.0;
  ImpedanceInput rotation_input;
  rotation_input.jacobian(5, 0) = 1.0;
  const Eigen::Quaterniond desired(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ()));

  const auto torque_for = [&](Eigen::Quaterniond orientation) {
    CartesianImpedanceCore rotation_core;
    ImpedanceTarget rotation_target;
    rotation_core.reset(rotation_input, rotation_target, rotation_params);
    rotation_target.orientation = orientation;
    ++rotation_target.sequence;
    (void)rotation_core.update(rotation_input, rotation_target, rotation_params);
    return rotation_core.update(rotation_input, rotation_target, rotation_params);
  };

  const Vector7d positive_quaternion_torque = torque_for(desired);
  Eigen::Quaterniond equivalent_negative = desired;
  equivalent_negative.coeffs() = -equivalent_negative.coeffs();
  const Vector7d negative_quaternion_torque = torque_for(equivalent_negative);
  if ((positive_quaternion_torque - negative_quaternion_torque).norm() >= 1e-12 ||
      positive_quaternion_torque(0) <= 0.0) {
    return 1;
  }

  return 0;
}
