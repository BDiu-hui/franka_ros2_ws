#include <cmath>
#include <iostream>
#include <string>

#include <serl_franka_controllers_ros2/cartesian_impedance_core.hpp>

namespace {

using serl_franka_controllers::CartesianImpedanceCore;
using serl_franka_controllers::ComplianceParams;
using serl_franka_controllers::ImpedanceInput;
using serl_franka_controllers::ImpedanceTarget;
using serl_franka_controllers::Vector7d;

int failures = 0;

void expect_near(const std::string& name, double actual, double expected, double tolerance = 1e-9) {
  if (std::abs(actual - expected) > tolerance) {
    std::cerr << name << ": expected " << expected << ", got " << actual << '\n';
    ++failures;
  }
}

void expect_vector_near(const std::string& name,
                        const Vector7d& actual,
                        const Vector7d& expected,
                        double tolerance = 1e-9) {
  if ((actual - expected).cwiseAbs().maxCoeff() > tolerance) {
    std::cerr << name << ": expected " << expected.transpose() << ", got "
              << actual.transpose() << '\n';
    ++failures;
  }
}

ComplianceParams task_only_params() {
  ComplianceParams params;
  params.nullspace_stiffness = 0.0;
  params.joint1_nullspace_stiffness = 0.0;
  params.translational_ki = 0.0;
  params.rotational_ki = 0.0;
  params.filter_params = 1.0;
  return params;
}

Vector7d apply_target_until_settled(CartesianImpedanceCore& core,
                                    ImpedanceInput& input,
                                    const ImpedanceTarget& target,
                                    const ComplianceParams& params,
                                    int cycles) {
  Vector7d torque = input.tau_j_d;
  for (int cycle = 0; cycle < cycles; ++cycle) {
    torque = core.update(input, target, params);
    input.tau_j_d = torque;
  }
  return torque;
}

void test_defaults_match_impedance_elbow_ros2() {
  const ComplianceParams params;
  expect_near("default translational stiffness", params.translational_stiffness, 2020.0);
  expect_near("default translational damping", params.translational_damping, 89.0);
  expect_near("default rotational stiffness", params.rotational_stiffness, 300.0);
  expect_near("default rotational damping", params.rotational_damping, 7.0);
  expect_near("default translation clip", params.translational_clip_x, 0.03);
  expect_near("default rotation clip", params.rotational_clip_x, 0.05);
  expect_near("default target filter", params.filter_params, 0.02);
}

void test_equilibrium_only_outputs_coriolis() {
  const ComplianceParams params = task_only_params();
  ImpedanceInput input;
  input.coriolis.setConstant(0.25);
  input.tau_j_d = input.coriolis;
  const ImpedanceTarget target;

  CartesianImpedanceCore core;
  core.reset(input, target, params);
  expect_vector_near("equilibrium coriolis compensation", core.update(input, target, params),
                     input.coriolis);
}

void test_translation_stiffness_on_all_axes() {
  const ComplianceParams params = task_only_params();
  for (int axis = 0; axis < 3; ++axis) {
    ImpedanceInput input;
    input.jacobian(axis, axis) = 1.0;
    ImpedanceTarget target;

    CartesianImpedanceCore core;
    core.reset(input, target, params);
    target.position(axis) = 0.005;
    ++target.sequence;
    const Vector7d torque = apply_target_until_settled(core, input, target, params, 20);
    expect_near("5 mm translation axis " + std::to_string(axis), torque(axis), 10.1);
  }
}

void test_translation_clip_limits_spring_wrench() {
  const ComplianceParams params = task_only_params();
  ImpedanceInput input;
  input.jacobian(0, 0) = 1.0;
  ImpedanceTarget target;

  CartesianImpedanceCore core;
  core.reset(input, target, params);
  target.position.x() = 0.1;
  ++target.sequence;
  const Vector7d torque = apply_target_until_settled(core, input, target, params, 70);
  expect_near("translation clip produces 2020 * 0.03 N", torque(0), 60.6);
}

void test_translation_damping() {
  ComplianceParams params = task_only_params();
  params.translational_stiffness = 0.0;
  ImpedanceInput input;
  input.jacobian(0, 0) = 1.0;
  input.dq(0) = 0.01;
  const ImpedanceTarget target;

  CartesianImpedanceCore core;
  core.reset(input, target, params);
  expect_near("0.01 m/s translation damping", core.update(input, target, params)(0), -0.89);
}

void test_rotation_stiffness_and_quaternion_sign() {
  ComplianceParams params = task_only_params();
  params.translational_stiffness = 0.0;
  ImpedanceInput input;
  input.jacobian(5, 0) = 1.0;
  const Eigen::Quaterniond desired(Eigen::AngleAxisd(0.02, Eigen::Vector3d::UnitZ()));

  const auto torque_for = [&](Eigen::Quaterniond orientation) {
    CartesianImpedanceCore core;
    ImpedanceTarget target;
    core.reset(input, target, params);
    target.orientation = orientation;
    ++target.sequence;
    return apply_target_until_settled(core, input, target, params, 5);
  };

  const Vector7d positive = torque_for(desired);
  Eigen::Quaterniond equivalent_negative = desired;
  equivalent_negative.coeffs() = -equivalent_negative.coeffs();
  const Vector7d negative = torque_for(equivalent_negative);
  expect_near("0.02 rad rotation stiffness", positive(0), 300.0 * std::sin(0.01));
  expect_vector_near("equivalent quaternion signs", negative, positive);
}

void test_target_filter_response() {
  ComplianceParams params = task_only_params();
  params.filter_params = 0.02;
  ImpedanceInput input;
  input.jacobian(0, 0) = 1.0;
  ImpedanceTarget target;

  CartesianImpedanceCore core;
  core.reset(input, target, params);
  target.position.x() = 0.005;
  ++target.sequence;
  const Vector7d torque = apply_target_until_settled(core, input, target, params, 51);
  const double expected = 2020.0 * 0.005 * (1.0 - std::pow(0.98, 50));
  expect_near("0.02 target filter after 50 ms", torque(0), expected, 1e-8);
}

void test_one_newton_meter_per_cycle_rate_limit() {
  const ComplianceParams params = task_only_params();
  ImpedanceInput input;
  input.jacobian(0, 0) = 1.0;
  ImpedanceTarget target;

  CartesianImpedanceCore core;
  core.reset(input, target, params);
  target.position.x() = 0.03;
  ++target.sequence;
  (void)core.update(input, target, params);
  const Vector7d first_limited = core.update(input, target, params);
  expect_near("first torque-rate-limited cycle", first_limited(0), 1.0);
  input.tau_j_d = first_limited;
  expect_near("second torque-rate-limited cycle", core.update(input, target, params)(0), 2.0);
}

}  // namespace

int main() {
  test_defaults_match_impedance_elbow_ros2();
  test_equilibrium_only_outputs_coriolis();
  test_translation_stiffness_on_all_axes();
  test_translation_clip_limits_spring_wrench();
  test_translation_damping();
  test_rotation_stiffness_and_quaternion_sign();
  test_target_filter_response();
  test_one_newton_meter_per_cycle_rate_limit();

  if (failures != 0) {
    std::cerr << failures << " offline impedance test(s) failed\n";
    return 1;
  }
  std::cout << "All offline impedance force tests passed\n";
  return 0;
}
