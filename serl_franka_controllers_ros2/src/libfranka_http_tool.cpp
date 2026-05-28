#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include <Eigen/Geometry>

#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

namespace {

constexpr std::array<double, 7> kDefaultLowerTorqueThresholdsNominal{
    {20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}};
constexpr std::array<double, 7> kDefaultUpperTorqueThresholdsNominal{
    {20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}};
constexpr std::array<double, 7> kDefaultLowerTorqueThresholdsAcceleration{
    {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}};
constexpr std::array<double, 7> kDefaultUpperTorqueThresholdsAcceleration{
    {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}};
constexpr std::array<double, 6> kDefaultLowerForceThresholdsNominal{{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}};
constexpr std::array<double, 6> kDefaultUpperForceThresholdsNominal{{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}};
constexpr std::array<double, 6> kDefaultLowerForceThresholdsAcceleration{
    {10.0, 10.0, 10.0, 10.0, 10.0, 10.0}};
constexpr std::array<double, 6> kDefaultUpperForceThresholdsAcceleration{
    {10.0, 10.0, 10.0, 10.0, 10.0, 10.0}};

template <size_t Size>
std::array<double, Size> read_array_env(const char* name, const std::array<double, Size>& defaults) {
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

void set_default_behavior(franka::Robot& robot) {
  const auto lower_torque_thresholds_acceleration = read_array_env(
      "LIBFRANKA_HTTP_LOWER_TORQUE_THRESHOLDS_ACCELERATION",
      kDefaultLowerTorqueThresholdsAcceleration);
  const auto upper_torque_thresholds_acceleration = read_array_env(
      "LIBFRANKA_HTTP_UPPER_TORQUE_THRESHOLDS_ACCELERATION",
      kDefaultUpperTorqueThresholdsAcceleration);
  const auto lower_torque_thresholds_nominal = read_array_env(
      "LIBFRANKA_HTTP_LOWER_TORQUE_THRESHOLDS_NOMINAL",
      kDefaultLowerTorqueThresholdsNominal);
  const auto upper_torque_thresholds_nominal = read_array_env(
      "LIBFRANKA_HTTP_UPPER_TORQUE_THRESHOLDS_NOMINAL",
      kDefaultUpperTorqueThresholdsNominal);
  const auto lower_force_thresholds_acceleration = read_array_env(
      "LIBFRANKA_HTTP_LOWER_FORCE_THRESHOLDS_ACCELERATION",
      kDefaultLowerForceThresholdsAcceleration);
  const auto upper_force_thresholds_acceleration = read_array_env(
      "LIBFRANKA_HTTP_UPPER_FORCE_THRESHOLDS_ACCELERATION",
      kDefaultUpperForceThresholdsAcceleration);
  const auto lower_force_thresholds_nominal = read_array_env(
      "LIBFRANKA_HTTP_LOWER_FORCE_THRESHOLDS_NOMINAL",
      kDefaultLowerForceThresholdsNominal);
  const auto upper_force_thresholds_nominal = read_array_env(
      "LIBFRANKA_HTTP_UPPER_FORCE_THRESHOLDS_NOMINAL",
      kDefaultUpperForceThresholdsNominal);

  robot.setCollisionBehavior(lower_torque_thresholds_acceleration,
                             upper_torque_thresholds_acceleration,
                             lower_torque_thresholds_nominal,
                             upper_torque_thresholds_nominal,
                             lower_force_thresholds_acceleration,
                             upper_force_thresholds_acceleration,
                             lower_force_thresholds_nominal,
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

std::string json_ok(const std::string& message) {
  return "{\"ok\":true,\"message\":\"" + message + "\"}";
}

std::string json_error(const std::string& error) {
  return "{\"ok\":false,\"error\":\"" + error + "\"}";
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
      [&start, &start_q, &target, &target_q, &elapsed,
       duration](const franka::RobotState& robot_state,
                 franka::Duration period) -> franka::CartesianPose {
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

}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc < 3) {
      std::cerr
          << "Usage:\n"
          << "  " << argv[0] << " get_pose <robot_ip>\n"
          << "  " << argv[0] << " get_state <robot_ip>\n"
          << "  " << argv[0] << " clear_error <robot_ip>\n"
          << "  " << argv[0] << " set_collision <robot_ip>\n"
          << "  " << argv[0]
          << " move_pose <robot_ip> <x> <y> <z> <qx> <qy> <qz> <qw> [duration_sec] [controller_mode]\n";
      return 2;
    }

    const std::string command = argv[1];
    const std::string robot_ip = argv[2];
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
