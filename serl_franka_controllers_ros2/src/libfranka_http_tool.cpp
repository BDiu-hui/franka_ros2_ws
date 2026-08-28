#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <thread>

#include <Eigen/Geometry>

#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include <realtime_tools/realtime_buffer.hpp>
#include <serl_franka_controllers_ros2/cartesian_impedance_core.hpp>

namespace {

constexpr std::array<double, 7> kDefaultLowerTorqueThresholdsNominal{
    {20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}};
constexpr std::array<double, 7> kDefaultUpperTorqueThresholdsNominal{
    {20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}};
constexpr std::array<double, 7> kDefaultLowerTorqueThresholdsAcceleration{
    {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}};
constexpr std::array<double, 7> kDefaultUpperTorqueThresholdsAcceleration{
    {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}};
constexpr std::array<double, 6> kDefaultLowerForceThresholdsNominal{
    {20.0, 20.0, 20.0, 20.0, 20.0, 20.0}};
constexpr std::array<double, 6> kDefaultUpperForceThresholdsNominal{
    {20.0, 20.0, 20.0, 20.0, 20.0, 20.0}};
constexpr std::array<double, 6> kDefaultLowerForceThresholdsAcceleration{
    {10.0, 10.0, 10.0, 10.0, 10.0, 10.0}};
constexpr std::array<double, 6> kDefaultUpperForceThresholdsAcceleration{
    {10.0, 10.0, 10.0, 10.0, 10.0, 10.0}};

using serl_franka_controllers::CartesianImpedanceCore;
using serl_franka_controllers::ComplianceParams;
using serl_franka_controllers::ElbowInput;
using serl_franka_controllers::ImpedanceInput;
using serl_franka_controllers::ImpedanceTarget;
using serl_franka_controllers::Matrix67d;
using serl_franka_controllers::Vector7d;

struct HttpStateSnapshot {
  std::array<double, 7> pose{};
  std::array<double, 6> velocity{};
  std::array<double, 3> force{};
  std::array<double, 3> torque{};
  std::array<double, 7> q{};
  std::array<double, 7> dq{};
  std::array<double, 42> jacobian{};
};

template <size_t Size>
class AtomicArraySnapshot {
 public:
  AtomicArraySnapshot() {
    for (auto& value : values_) {
      value.store(0.0, std::memory_order_relaxed);
    }
  }

  void store(const std::array<double, Size>& values) {
    version_.fetch_add(1, std::memory_order_acq_rel);
    for (size_t index = 0; index < Size; ++index) {
      values_[index].store(values[index], std::memory_order_relaxed);
    }
    version_.fetch_add(1, std::memory_order_release);
  }

  [[nodiscard]] bool load(std::array<double, Size>& values) const {
    while (true) {
      const std::uint64_t before = version_.load(std::memory_order_acquire);
      if (before == 0) {
        return false;
      }
      if ((before & 1U) != 0U) {
        continue;
      }
      for (size_t index = 0; index < Size; ++index) {
        values[index] = values_[index].load(std::memory_order_relaxed);
      }
      if (before == version_.load(std::memory_order_acquire)) {
        return true;
      }
    }
  }

 private:
  mutable std::atomic<std::uint64_t> version_{0};
  std::array<std::atomic<double>, Size> values_{};
};

class AtomicHttpState {
 public:
  void store(const HttpStateSnapshot& state) {
    std::array<double, 75> values{};
    auto output = values.begin();
    output = std::copy(state.pose.begin(), state.pose.end(), output);
    output = std::copy(state.velocity.begin(), state.velocity.end(), output);
    output = std::copy(state.force.begin(), state.force.end(), output);
    output = std::copy(state.torque.begin(), state.torque.end(), output);
    output = std::copy(state.q.begin(), state.q.end(), output);
    output = std::copy(state.dq.begin(), state.dq.end(), output);
    std::copy(state.jacobian.begin(), state.jacobian.end(), output);
    values_.store(values);
  }

  [[nodiscard]] bool load(HttpStateSnapshot& state) const {
    std::array<double, 75> values{};
    if (!values_.load(values)) {
      return false;
    }
    auto input = values.cbegin();
    input = std::copy_n(input, state.pose.size(), state.pose.begin());
    input = std::copy_n(input, state.velocity.size(), state.velocity.begin());
    input = std::copy_n(input, state.force.size(), state.force.begin());
    input = std::copy_n(input, state.torque.size(), state.torque.begin());
    input = std::copy_n(input, state.q.size(), state.q.begin());
    input = std::copy_n(input, state.dq.size(), state.dq.begin());
    std::copy_n(input, state.jacobian.size(), state.jacobian.begin());
    return true;
  }

 private:
  AtomicArraySnapshot<75> values_;
};

template <size_t Size>
std::array<double, Size> read_array_env(const char* name,
                                        const std::array<double, Size>& defaults) {
  const char* raw_value = std::getenv(name);
  if (raw_value == nullptr || std::string(raw_value).empty()) {
    return defaults;
  }

  std::string normalized(raw_value);
  for (char& character : normalized) {
    if (character == '[' || character == ']' || character == ',') {
      character = ' ';
    }
  }

  std::array<double, Size> values{};
  std::istringstream input(normalized);
  for (size_t index = 0; index < Size; ++index) {
    if (!(input >> values[index])) {
      throw std::invalid_argument(std::string("Invalid collision threshold array in ") + name);
    }
  }

  double extra = 0.0;
  if (input >> extra) {
    throw std::invalid_argument(std::string("Too many collision threshold values in ") + name);
  }
  return values;
}

double read_double_env(const char* name, double default_value) {
  const char* raw_value = std::getenv(name);
  if (raw_value == nullptr || *raw_value == '\0') {
    return default_value;
  }
  char* end = nullptr;
  const double value = std::strtod(raw_value, &end);
  if (end == raw_value || (end != nullptr && *end != '\0') || !std::isfinite(value)) {
    throw std::invalid_argument(std::string(name) + " must be finite");
  }
  return value;
}

ComplianceParams read_impedance_params_env() {
  ComplianceParams params;
  struct ParamSpec {
    const char* name;
    double ComplianceParams::*member;
  };
  static constexpr ParamSpec kSpecs[] = {
      {"LIBFRANKA_HTTP_TRANSLATIONAL_STIFFNESS", &ComplianceParams::translational_stiffness},
      {"LIBFRANKA_HTTP_TRANSLATIONAL_DAMPING", &ComplianceParams::translational_damping},
      {"LIBFRANKA_HTTP_ROTATIONAL_STIFFNESS", &ComplianceParams::rotational_stiffness},
      {"LIBFRANKA_HTTP_ROTATIONAL_DAMPING", &ComplianceParams::rotational_damping},
      {"LIBFRANKA_HTTP_NULLSPACE_STIFFNESS", &ComplianceParams::nullspace_stiffness},
      {"LIBFRANKA_HTTP_JOINT1_NULLSPACE_STIFFNESS", &ComplianceParams::joint1_nullspace_stiffness},
      {"LIBFRANKA_HTTP_TRANSLATIONAL_CLIP_NEG_X", &ComplianceParams::translational_clip_neg_x},
      {"LIBFRANKA_HTTP_TRANSLATIONAL_CLIP_NEG_Y", &ComplianceParams::translational_clip_neg_y},
      {"LIBFRANKA_HTTP_TRANSLATIONAL_CLIP_NEG_Z", &ComplianceParams::translational_clip_neg_z},
      {"LIBFRANKA_HTTP_TRANSLATIONAL_CLIP_X", &ComplianceParams::translational_clip_x},
      {"LIBFRANKA_HTTP_TRANSLATIONAL_CLIP_Y", &ComplianceParams::translational_clip_y},
      {"LIBFRANKA_HTTP_TRANSLATIONAL_CLIP_Z", &ComplianceParams::translational_clip_z},
      {"LIBFRANKA_HTTP_ROTATIONAL_CLIP_NEG_X", &ComplianceParams::rotational_clip_neg_x},
      {"LIBFRANKA_HTTP_ROTATIONAL_CLIP_NEG_Y", &ComplianceParams::rotational_clip_neg_y},
      {"LIBFRANKA_HTTP_ROTATIONAL_CLIP_NEG_Z", &ComplianceParams::rotational_clip_neg_z},
      {"LIBFRANKA_HTTP_ROTATIONAL_CLIP_X", &ComplianceParams::rotational_clip_x},
      {"LIBFRANKA_HTTP_ROTATIONAL_CLIP_Y", &ComplianceParams::rotational_clip_y},
      {"LIBFRANKA_HTTP_ROTATIONAL_CLIP_Z", &ComplianceParams::rotational_clip_z},
      {"LIBFRANKA_HTTP_TRANSLATIONAL_KI", &ComplianceParams::translational_ki},
      {"LIBFRANKA_HTTP_ROTATIONAL_KI", &ComplianceParams::rotational_ki},
      {"LIBFRANKA_HTTP_FILTER_PARAMS", &ComplianceParams::filter_params},
      {"LIBFRANKA_HTTP_ELBOW_STIFFNESS", &ComplianceParams::elbow_stiffness},
      {"LIBFRANKA_HTTP_ELBOW_DAMPING", &ComplianceParams::elbow_damping},
  };
  for (const auto& spec : kSpecs) {
    params.*(spec.member) = read_double_env(spec.name, params.*(spec.member));
  }

  if (params.translational_stiffness < 0.0 || params.translational_damping < 0.0 ||
      params.rotational_stiffness < 0.0 || params.rotational_damping < 0.0 ||
      params.nullspace_stiffness < 0.0 || params.joint1_nullspace_stiffness < 0.0 ||
      params.elbow_stiffness < 0.0 || params.elbow_damping < 0.0 ||
      params.translational_clip_neg_x < 0.0 || params.translational_clip_neg_y < 0.0 ||
      params.translational_clip_neg_z < 0.0 || params.translational_clip_x < 0.0 ||
      params.translational_clip_y < 0.0 || params.translational_clip_z < 0.0 ||
      params.rotational_clip_neg_x < 0.0 || params.rotational_clip_neg_y < 0.0 ||
      params.rotational_clip_neg_z < 0.0 || params.rotational_clip_x < 0.0 ||
      params.rotational_clip_y < 0.0 || params.rotational_clip_z < 0.0 ||
      params.filter_params < 0.0 || params.filter_params > 1.0) {
    throw std::invalid_argument(
        "Impedance stiffness, damping, clips, and filter must be non-negative");
  }
  return params;
}

void set_default_behavior(franka::Robot& robot) {
  const auto lower_torque_thresholds_acceleration =
      read_array_env("LIBFRANKA_HTTP_LOWER_TORQUE_THRESHOLDS_ACCELERATION",
                     kDefaultLowerTorqueThresholdsAcceleration);
  const auto upper_torque_thresholds_acceleration =
      read_array_env("LIBFRANKA_HTTP_UPPER_TORQUE_THRESHOLDS_ACCELERATION",
                     kDefaultUpperTorqueThresholdsAcceleration);
  const auto lower_torque_thresholds_nominal = read_array_env(
      "LIBFRANKA_HTTP_LOWER_TORQUE_THRESHOLDS_NOMINAL", kDefaultLowerTorqueThresholdsNominal);
  const auto upper_torque_thresholds_nominal = read_array_env(
      "LIBFRANKA_HTTP_UPPER_TORQUE_THRESHOLDS_NOMINAL", kDefaultUpperTorqueThresholdsNominal);
  const auto lower_force_thresholds_acceleration =
      read_array_env("LIBFRANKA_HTTP_LOWER_FORCE_THRESHOLDS_ACCELERATION",
                     kDefaultLowerForceThresholdsAcceleration);
  const auto upper_force_thresholds_acceleration =
      read_array_env("LIBFRANKA_HTTP_UPPER_FORCE_THRESHOLDS_ACCELERATION",
                     kDefaultUpperForceThresholdsAcceleration);
  const auto lower_force_thresholds_nominal = read_array_env(
      "LIBFRANKA_HTTP_LOWER_FORCE_THRESHOLDS_NOMINAL", kDefaultLowerForceThresholdsNominal);
  const auto upper_force_thresholds_nominal = read_array_env(
      "LIBFRANKA_HTTP_UPPER_FORCE_THRESHOLDS_NOMINAL", kDefaultUpperForceThresholdsNominal);

  robot.setCollisionBehavior(lower_torque_thresholds_acceleration,
                             upper_torque_thresholds_acceleration, lower_torque_thresholds_nominal,
                             upper_torque_thresholds_nominal, lower_force_thresholds_acceleration,
                             upper_force_thresholds_acceleration, lower_force_thresholds_nominal,
                             upper_force_thresholds_nominal);
  robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
}

Eigen::Isometry3d pose_array_to_isometry(const std::array<double, 7>& pose) {
  Eigen::Quaterniond quaternion(pose[6], pose[3], pose[4], pose[5]);
  quaternion.normalize();

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(pose[0], pose[1], pose[2]);
  transform.linear() = quaternion.toRotationMatrix();
  return transform;
}

std::array<double, 7> isometry_to_pose_array(const Eigen::Isometry3d& transform) {
  Eigen::Quaterniond quaternion(transform.linear());
  quaternion.normalize();
  return {{
      transform.translation().x(),
      transform.translation().y(),
      transform.translation().z(),
      quaternion.x(),
      quaternion.y(),
      quaternion.z(),
      quaternion.w(),
  }};
}

Eigen::Isometry3d robot_state_to_isometry(const franka::RobotState& robot_state) {
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      matrix(row, col) = robot_state.O_T_EE[col * 4 + row];
    }
  }
  return Eigen::Isometry3d(matrix);
}

std::array<double, 16> isometry_to_matrix_array(const Eigen::Isometry3d& transform) {
  Eigen::Matrix4d matrix = transform.matrix();
  std::array<double, 16> result{};
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      result[col * 4 + row] = matrix(row, col);
    }
  }
  return result;
}

double smoothstep_5(double s) {
  if (s <= 0.0) {
    return 0.0;
  }
  if (s >= 1.0) {
    return 1.0;
  }
  return s * s * s * (10.0 + s * (-15.0 + 6.0 * s));
}

std::string json_pose(const std::array<double, 7>& pose) {
  std::ostringstream output;
  output << std::fixed << std::setprecision(9);
  output << "{\"ok\":true,\"pose\":[";
  for (size_t index = 0; index < pose.size(); ++index) {
    if (index > 0) {
      output << ",";
    }
    output << pose[index];
  }
  output << "]}";
  return output.str();
}

template <size_t Size>
void append_array(std::ostringstream& output, const std::array<double, Size>& values) {
  output << "[";
  for (size_t index = 0; index < values.size(); ++index) {
    if (index > 0) {
      output << ",";
    }
    output << values[index];
  }
  output << "]";
}

void append_jacobian(std::ostringstream& output, const std::array<double, 42>& jacobian) {
  output << "[";
  for (size_t row = 0; row < 6; ++row) {
    if (row > 0) {
      output << ",";
    }
    output << "[";
    for (size_t col = 0; col < 7; ++col) {
      if (col > 0) {
        output << ",";
      }
      output << jacobian[row + 6 * col];
    }
    output << "]";
  }
  output << "]";
}

std::array<double, 6> jacobian_times_dq(const std::array<double, 42>& jacobian,
                                        const std::array<double, 7>& dq) {
  std::array<double, 6> velocity{};
  for (size_t row = 0; row < 6; ++row) {
    for (size_t col = 0; col < 7; ++col) {
      velocity[row] += jacobian[row + 6 * col] * dq[col];
    }
  }
  return velocity;
}

std::string json_state(const franka::RobotState& robot_state,
                       const std::array<double, 42>& jacobian) {
  const auto pose = isometry_to_pose_array(robot_state_to_isometry(robot_state));
  const auto velocity = jacobian_times_dq(jacobian, robot_state.dq);
  const std::array<double, 3> force{{
      robot_state.K_F_ext_hat_K[0],
      robot_state.K_F_ext_hat_K[1],
      robot_state.K_F_ext_hat_K[2],
  }};
  const std::array<double, 3> torque{{
      robot_state.K_F_ext_hat_K[3],
      robot_state.K_F_ext_hat_K[4],
      robot_state.K_F_ext_hat_K[5],
  }};

  std::ostringstream output;
  output << std::fixed << std::setprecision(9);
  output << "{\"ok\":true,\"pose\":";
  append_array(output, pose);
  output << ",\"vel\":";
  append_array(output, velocity);
  output << ",\"force\":";
  append_array(output, force);
  output << ",\"torque\":";
  append_array(output, torque);
  output << ",\"q\":";
  append_array(output, robot_state.q);
  output << ",\"dq\":";
  append_array(output, robot_state.dq);
  output << ",\"jacobian\":";
  append_jacobian(output, jacobian);
  output << ",\"gripper_pos\":null,\"have_gripper\":false}";
  return output.str();
}

std::string json_state(const HttpStateSnapshot& state) {
  std::ostringstream output;
  output << std::fixed << std::setprecision(9);
  output << "{\"ok\":true,\"pose\":";
  append_array(output, state.pose);
  output << ",\"vel\":";
  append_array(output, state.velocity);
  output << ",\"force\":";
  append_array(output, state.force);
  output << ",\"torque\":";
  append_array(output, state.torque);
  output << ",\"q\":";
  append_array(output, state.q);
  output << ",\"dq\":";
  append_array(output, state.dq);
  output << ",\"jacobian\":";
  append_jacobian(output, state.jacobian);
  output << ",\"gripper_pos\":null,\"have_gripper\":false}";
  return output.str();
}

std::string json_string(const std::string& value) {
  std::ostringstream output;
  output << '"';
  for (const unsigned char character : value) {
    switch (character) {
      case '"':
        output << "\\\"";
        break;
      case '\\':
        output << "\\\\";
        break;
      case '\b':
        output << "\\b";
        break;
      case '\f':
        output << "\\f";
        break;
      case '\n':
        output << "\\n";
        break;
      case '\r':
        output << "\\r";
        break;
      case '\t':
        output << "\\t";
        break;
      default:
        if (character < 0x20) {
          output << "\\u00" << std::hex << std::setw(2) << std::setfill('0')
                 << static_cast<int>(character) << std::dec << std::setfill(' ');
        } else {
          output << character;
        }
    }
  }
  output << '"';
  return output.str();
}

std::string json_ok(const std::string& message) {
  return "{\"ok\":true,\"message\":" + json_string(message) + "}";
}

std::string json_error(const std::string& error) {
  return "{\"ok\":false,\"error\":" + json_string(error) + "}";
}

double parse_double(const char* value, const std::string& name) {
  char* end = nullptr;
  const double parsed = std::strtod(value, &end);
  if (end == value || (end != nullptr && *end != '\0')) {
    throw std::invalid_argument("Invalid numeric value for " + name);
  }
  return parsed;
}

franka::ControllerMode parse_controller_mode(const std::string& value) {
  if (value == "joint") {
    return franka::ControllerMode::kJointImpedance;
  }
  if (value == "cartesian") {
    return franka::ControllerMode::kCartesianImpedance;
  }
  throw std::invalid_argument("controller_mode must be 'joint' or 'cartesian'");
}

double compute_duration(const Eigen::Isometry3d& start,
                        const Eigen::Isometry3d& target,
                        double linear_speed,
                        double angular_speed) {
  const double distance = (target.translation() - start.translation()).norm();
  const Eigen::Quaterniond start_q(start.linear());
  const Eigen::Quaterniond target_q(target.linear());
  const double angular_distance = start_q.angularDistance(target_q);
  const double linear_duration = distance / std::max(1e-6, linear_speed);
  const double angular_duration = angular_distance / std::max(1e-6, angular_speed);
  return std::max(0.5, std::max(linear_duration, angular_duration));
}

void move_pose(franka::Robot& robot,
               const std::array<double, 7>& target_pose,
               double duration,
               franka::ControllerMode controller_mode) {
  set_default_behavior(robot);
  const Eigen::Isometry3d target = pose_array_to_isometry(target_pose);

  Eigen::Isometry3d start = Eigen::Isometry3d::Identity();
  Eigen::Quaterniond start_q = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond target_q(target.linear());
  double elapsed = 0.0;

  robot.control(
      [&start, &start_q, &target, &target_q, &elapsed, duration](
          const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianPose {
        if (elapsed == 0.0) {
          start = robot_state_to_isometry(robot_state);
          start_q = Eigen::Quaterniond(start.linear());
        }

        elapsed += period.toSec();
        const double alpha = smoothstep_5(elapsed / duration);

        Eigen::Isometry3d interpolated = Eigen::Isometry3d::Identity();
        interpolated.translation() =
            start.translation() + alpha * (target.translation() - start.translation());
        interpolated.linear() = start_q.slerp(alpha, target_q).normalized().toRotationMatrix();

        const std::array<double, 16> matrix = isometry_to_matrix_array(interpolated);
        if (elapsed >= duration) {
          return franka::MotionFinished(matrix);
        }
        return matrix;
      },
      controller_mode);
}

HttpStateSnapshot make_http_state(const franka::RobotState& robot_state,
                                  const std::array<double, 42>& jacobian) {
  HttpStateSnapshot state;
  state.pose = isometry_to_pose_array(robot_state_to_isometry(robot_state));
  state.velocity = jacobian_times_dq(jacobian, robot_state.dq);
  std::copy_n(robot_state.K_F_ext_hat_K.begin(), 3, state.force.begin());
  std::copy_n(robot_state.K_F_ext_hat_K.begin() + 3, 3, state.torque.begin());
  state.q = robot_state.q;
  state.dq = robot_state.dq;
  state.jacobian = jacobian;
  return state;
}

ImpedanceInput make_impedance_input(const franka::RobotState& robot_state,
                                    const std::array<double, 7>& coriolis,
                                    const std::array<double, 42>& jacobian) {
  ImpedanceInput input;
  input.q = Eigen::Map<const Vector7d>(robot_state.q.data());
  input.dq = Eigen::Map<const Vector7d>(robot_state.dq.data());
  input.tau_j_d = Eigen::Map<const Vector7d>(robot_state.tau_J_d.data());
  input.coriolis = Eigen::Map<const Vector7d>(coriolis.data());
  input.jacobian = Eigen::Map<const Matrix67d>(jacobian.data());
  input.ee_pose = robot_state_to_isometry(robot_state);
  return input;
}

enum class WorkerStatus { kStarting, kRunning, kStopped, kFailed };

int run_impedance_server(const std::string& robot_ip) {
  const ComplianceParams params = read_impedance_params_env();
  realtime_tools::RealtimeBuffer<ImpedanceTarget> target_buffer;
  AtomicHttpState state_buffer;
  std::atomic<bool> stop_requested{false};
  std::atomic<WorkerStatus> status{WorkerStatus::kStarting};
  std::string control_error;

  std::thread control_thread([&]() {
    try {
      franka::Robot robot(robot_ip, franka::RealtimeConfig::kEnforce);
      set_default_behavior(robot);
      franka::Model model = robot.loadModel();
      const franka::RobotState initial_state = robot.readOnce();
      const auto initial_jacobian = model.zeroJacobian(franka::Frame::kEndEffector, initial_state);
      const ImpedanceInput initial_input =
          make_impedance_input(initial_state, model.coriolis(initial_state), initial_jacobian);
      ImpedanceTarget initial_target;
      initial_target.position = initial_input.ee_pose.translation();
      initial_target.orientation = Eigen::Quaterniond(initial_input.ee_pose.linear());
      initial_target.master_q = initial_input.q;
      target_buffer.initRT(initial_target);

      CartesianImpedanceCore core;
      core.reset(initial_input, initial_target, params);
      state_buffer.store(make_http_state(initial_state, initial_jacobian));
      status.store(WorkerStatus::kRunning, std::memory_order_release);
      size_t state_publish_counter = 0;

      robot.control([&](const franka::RobotState& robot_state,
                        franka::Duration /*period*/) -> franka::Torques {
        const auto jacobian = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
        const ImpedanceInput input =
            make_impedance_input(robot_state, model.coriolis(robot_state), jacobian);
        const ImpedanceTarget target = *target_buffer.readFromRT();
        std::optional<ElbowInput> elbow;
        if (core.needsElbowInput(target, params)) {
          const auto elbow_pose = model.pose(franka::Frame::kJoint4, robot_state);
          const auto elbow_jacobian = model.zeroJacobian(franka::Frame::kJoint4, robot_state);
          elbow.emplace();
          elbow->position =
              Eigen::Isometry3d(Eigen::Matrix4d::Map(elbow_pose.data())).translation();
          elbow->jacobian = Eigen::Map<const Matrix67d>(elbow_jacobian.data()).topRows(3);
        }

        const Vector7d torque = core.update(input, target, params, elbow);
        std::array<double, 7> torque_array{};
        Eigen::Map<Vector7d>(torque_array.data()) = torque;
        if (++state_publish_counter >= 10) {
          state_publish_counter = 0;
          state_buffer.store(make_http_state(robot_state, jacobian));
        }
        if (stop_requested.load(std::memory_order_relaxed)) {
          return franka::MotionFinished(franka::Torques(torque_array));
        }
        return franka::Torques(torque_array);
      });
      status.store(WorkerStatus::kStopped, std::memory_order_release);
    } catch (const std::exception& exception) {
      control_error = exception.what();
      status.store(WorkerStatus::kFailed, std::memory_order_release);
    }
  });

  while (status.load(std::memory_order_acquire) == WorkerStatus::kStarting) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (status.load(std::memory_order_acquire) == WorkerStatus::kFailed) {
    control_thread.join();
    std::cout << json_error(control_error) << std::endl;
    return 1;
  }
  std::cout << "{\"ok\":true,\"message\":\"Impedance control ready\"}" << std::endl;

  std::string line;
  while (std::getline(std::cin, line)) {
    try {
      std::istringstream input(line);
      std::string command;
      input >> command;
      if (command == "shutdown") {
        stop_requested.store(true, std::memory_order_relaxed);
        control_thread.join();
        std::cout << json_ok("Stopped") << std::endl;
        return 0;
      }
      if (status.load(std::memory_order_acquire) == WorkerStatus::kFailed) {
        throw std::runtime_error(control_error);
      }
      if (command == "health") {
        std::cout << json_ok("Impedance control active") << std::endl;
        continue;
      }
      if (command == "get_state" || command == "get_pose") {
        HttpStateSnapshot state;
        if (!state_buffer.load(state)) {
          throw std::runtime_error("Robot state is not available yet");
        }
        std::cout << (command == "get_pose" ? json_pose(state.pose) : json_state(state))
                  << std::endl;
        continue;
      }
      if (command == "pose") {
        std::array<double, 7> pose{};
        for (double& value : pose) {
          if (!(input >> value) || !std::isfinite(value)) {
            throw std::invalid_argument("pose requires 7 finite values");
          }
        }
        ImpedanceTarget target = *target_buffer.readFromNonRT();
        target.position << pose[0], pose[1], pose[2];
        target.orientation = Eigen::Quaterniond(pose[6], pose[3], pose[4], pose[5]);
        if (target.orientation.norm() < 1e-9) {
          throw std::invalid_argument("pose quaternion must be non-zero");
        }
        target.orientation.normalize();

        double master_q0;
        if (input >> master_q0) {
          target.master_q(0) = master_q0;
          for (Eigen::Index index = 1; index < target.master_q.size(); ++index) {
            if (!(input >> target.master_q(index)) || !std::isfinite(target.master_q(index))) {
              throw std::invalid_argument("master_q requires 7 finite values");
            }
          }
          target.has_master_q = true;
        } else {
          target.has_master_q = false;
        }
        std::string extra;
        if (input >> extra) {
          throw std::invalid_argument("pose command contains extra values");
        }
        ++target.sequence;
        target_buffer.writeFromNonRT(target);
        std::cout << json_ok("Target updated") << std::endl;
        continue;
      }
      throw std::invalid_argument("Unknown impedance command: " + command);
    } catch (const std::exception& exception) {
      std::cout << json_error(exception.what()) << std::endl;
    }
  }

  stop_requested.store(true, std::memory_order_relaxed);
  if (control_thread.joinable()) {
    control_thread.join();
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc < 3) {
      std::cerr << "Usage:\n"
                << "  " << argv[0] << " get_pose <robot_ip>\n"
                << "  " << argv[0] << " get_state <robot_ip>\n"
                << "  " << argv[0] << " clear_error <robot_ip>\n"
                << "  " << argv[0] << " set_collision <robot_ip>\n"
                << "  " << argv[0] << " impedance <robot_ip>\n"
                << "  " << argv[0]
                << " move_pose <robot_ip> <x> <y> <z> <qx> <qy> <qz> <qw> [duration_sec] "
                   "[controller_mode]\n";
      return 2;
    }

    const std::string command = argv[1];
    const std::string robot_ip = argv[2];
    if (command == "impedance") {
      return run_impedance_server(robot_ip);
    }
    franka::Robot robot(robot_ip, franka::RealtimeConfig::kIgnore);

    if (command == "get_pose") {
      const franka::RobotState robot_state = robot.readOnce();
      const auto pose = isometry_to_pose_array(robot_state_to_isometry(robot_state));
      std::cout << json_pose(pose) << std::endl;
      return 0;
    }

    if (command == "get_state") {
      const franka::RobotState robot_state = robot.readOnce();
      franka::Model model = robot.loadModel();
      const auto jacobian = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
      std::cout << json_state(robot_state, jacobian) << std::endl;
      return 0;
    }

    if (command == "clear_error") {
      robot.automaticErrorRecovery();
      std::cout << json_ok("Cleared") << std::endl;
      return 0;
    }

    if (command == "set_collision") {
      set_default_behavior(robot);
      std::cout << json_ok("Collision thresholds updated") << std::endl;
      return 0;
    }

    if (command == "move_pose") {
      if (argc < 10) {
        throw std::invalid_argument(
            "move_pose requires <x> <y> <z> <qx> <qy> <qz> <qw> [duration_sec] [controller_mode]");
      }

      std::array<double, 7> pose{};
      for (size_t index = 0; index < pose.size(); ++index) {
        pose[index] = parse_double(argv[3 + index], "pose");
      }

      const Eigen::Isometry3d target = pose_array_to_isometry(pose);
      const Eigen::Isometry3d current = robot_state_to_isometry(robot.readOnce());
      double duration = argc >= 11 ? parse_double(argv[10], "duration_sec")
                                   : compute_duration(current, target, 0.03, 0.25);
      duration = std::max(0.5, duration);
      const franka::ControllerMode controller_mode =
          argc >= 12 ? parse_controller_mode(argv[11]) : franka::ControllerMode::kJointImpedance;

      move_pose(robot, pose, duration, controller_mode);

      std::ostringstream output;
      output << std::fixed << std::setprecision(6);
      output << "{\"ok\":true,\"message\":\"Moved\",\"duration_sec\":" << duration
             << ",\"controller_mode\":\""
             << (controller_mode == franka::ControllerMode::kJointImpedance ? "joint" : "cartesian")
             << "\"}";
      std::cout << output.str() << std::endl;
      return 0;
    }

    throw std::invalid_argument("Unknown command: " + command);
  } catch (const std::exception& exception) {
    std::cerr << json_error(exception.what()) << std::endl;
    return 1;
  }
}
