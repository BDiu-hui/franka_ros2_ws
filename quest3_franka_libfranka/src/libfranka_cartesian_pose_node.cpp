#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

namespace {

using SteadyClock = std::chrono::steady_clock;

enum class ControlStatus : int {
  kConnecting = 0,
  kDisabled = 1,
  kWaitingForTarget = 2,
  kInitialTargetTooFar = 3,
  kRunning = 4,
  kException = 5,
  kStopped = 6,
};

const char* status_to_string(ControlStatus status) {
  switch (status) {
    case ControlStatus::kConnecting:
      return "connecting";
    case ControlStatus::kDisabled:
      return "disabled";
    case ControlStatus::kWaitingForTarget:
      return "waiting_for_target";
    case ControlStatus::kInitialTargetTooFar:
      return "initial_target_too_far_from_current";
    case ControlStatus::kRunning:
      return "running";
    case ControlStatus::kException:
      return "exception";
    case ControlStatus::kStopped:
      return "stopped";
  }
  return "unknown";
}

Eigen::Isometry3d array_to_transform(const std::array<double, 16>& values) {
  Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::ColMajor>> matrix(values.data());
  Eigen::Isometry3d transform(matrix);
  return transform;
}

std::array<double, 16> transform_to_array(const Eigen::Isometry3d& transform) {
  std::array<double, 16> values{};
  Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(values.data()) =
      transform.matrix();
  return values;
}

Eigen::Vector3d clamp_position(
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& min_position,
    const Eigen::Vector3d& max_position) {
  return position.cwiseMax(min_position).cwiseMin(max_position);
}

double seconds_since(const SteadyClock::time_point& stamp, const SteadyClock::time_point& now) {
  if (stamp == SteadyClock::time_point{}) {
    return std::numeric_limits<double>::infinity();
  }
  return std::chrono::duration<double>(now - stamp).count();
}

}  // namespace

class LibfrankaCartesianPoseNode : public rclcpp::Node {
 public:
  LibfrankaCartesianPoseNode() : Node("libfranka_cartesian_pose") {
    declare_parameter<std::string>("robot_ip", "172.16.0.3");
    declare_parameter<std::string>("target_pose_topic", "/franka_sim/tcp_target_pose");
    declare_parameter<std::string>("enabled_topic", "/quest3/right_teleop/enabled");
    declare_parameter<std::string>("current_pose_topic", "/franka_libfranka/current_pose");
    declare_parameter<std::string>("debug_topic", "/franka_libfranka/debug");
    declare_parameter<std::string>("base_frame", "panda_link0");
    declare_parameter<double>("publish_rate_hz", 50.0);
    declare_parameter<double>("target_timeout_sec", 0.25);
    declare_parameter<double>("enabled_timeout_sec", 0.25);
    declare_parameter<double>("max_linear_velocity_mps", 0.04);
    declare_parameter<double>("max_angular_velocity_radps", 0.25);
    declare_parameter<double>("max_initial_target_distance_m", 0.08);
    declare_parameter<double>("max_initial_target_angle_rad", 0.6);
    declare_parameter<std::vector<double>>("workspace_min", {0.20, -0.45, 0.08});
    declare_parameter<std::vector<double>>("workspace_max", {0.80, 0.45, 0.75});
    declare_parameter<std::vector<std::string>>(
        "joint_names",
        {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
         "panda_joint5", "panda_joint6", "panda_joint7"});
    declare_parameter<bool>("automatic_error_recovery", true);

    robot_ip_ = get_parameter("robot_ip").as_string();
    target_pose_topic_ = get_parameter("target_pose_topic").as_string();
    enabled_topic_ = get_parameter("enabled_topic").as_string();
    current_pose_topic_ = get_parameter("current_pose_topic").as_string();
    debug_topic_ = get_parameter("debug_topic").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
    target_timeout_sec_ = get_parameter("target_timeout_sec").as_double();
    enabled_timeout_sec_ = get_parameter("enabled_timeout_sec").as_double();
    max_linear_velocity_mps_ = get_parameter("max_linear_velocity_mps").as_double();
    max_angular_velocity_radps_ = get_parameter("max_angular_velocity_radps").as_double();
    max_initial_target_distance_m_ = get_parameter("max_initial_target_distance_m").as_double();
    max_initial_target_angle_rad_ = get_parameter("max_initial_target_angle_rad").as_double();
    joint_names_ = get_parameter("joint_names").as_string_array();
    automatic_error_recovery_ = get_parameter("automatic_error_recovery").as_bool();

    workspace_min_ = load_vector3_parameter("workspace_min");
    workspace_max_ = load_vector3_parameter("workspace_max");
    if (joint_names_.size() != 7) {
      throw std::runtime_error("joint_names must contain exactly 7 names");
    }

    target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        target_pose_topic_, 10,
        std::bind(&LibfrankaCartesianPoseNode::target_pose_callback, this, std::placeholders::_1));
    enabled_sub_ = create_subscription<std_msgs::msg::Bool>(
        enabled_topic_, 10,
        std::bind(&LibfrankaCartesianPoseNode::enabled_callback, this, std::placeholders::_1));

    current_pose_pub_ =
        create_publisher<geometry_msgs::msg::PoseStamped>(current_pose_topic_, 10);
    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    debug_pub_ = create_publisher<std_msgs::msg::String>(debug_topic_, 10);

    const double publish_period = 1.0 / std::max(publish_rate_hz_, 1.0);
    publish_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(publish_period)),
        std::bind(&LibfrankaCartesianPoseNode::publish_state_timer, this));

    running_.store(true);
    control_thread_ = std::thread(&LibfrankaCartesianPoseNode::run_control, this);
    RCLCPP_INFO(
        get_logger(),
        "Direct libfranka Cartesian pose node started. robot_ip=%s target=%s enabled=%s",
        robot_ip_.c_str(), target_pose_topic_.c_str(), enabled_topic_.c_str());
  }

  ~LibfrankaCartesianPoseNode() override {
    running_.store(false);
    if (control_thread_.joinable()) {
      control_thread_.join();
    }
  }

 private:
  struct Target {
    bool valid{false};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
    SteadyClock::time_point stamp{};
  };

  struct CurrentState {
    bool valid{false};
    std::array<double, 16> pose{};
    std::array<double, 7> q{};
    std::array<double, 7> dq{};
    std::array<double, 7> tau_j{};
  };

  Eigen::Vector3d load_vector3_parameter(const std::string& name) {
    const auto values = get_parameter(name).as_double_array();
    if (values.size() != 3) {
      throw std::runtime_error(name + " must contain exactly 3 values");
    }
    return Eigen::Vector3d(values[0], values[1], values[2]);
  }

  void target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    Eigen::Quaterniond orientation(
        msg->pose.orientation.w,
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z);
    if (orientation.norm() < 1e-9) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring invalid target quaternion");
      return;
    }
    orientation.normalize();

    Target target;
    target.valid = true;
    target.position = Eigen::Vector3d(
        msg->pose.position.x,
        msg->pose.position.y,
        msg->pose.position.z);
    target.orientation = orientation;
    target.stamp = SteadyClock::now();

    std::lock_guard<std::mutex> lock(target_mutex_);
    target_ = target;
  }

  void enabled_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(target_mutex_);
    teleop_enabled_ = msg->data;
    enabled_stamp_ = SteadyClock::now();
  }

  void run_control() {
    status_.store(static_cast<int>(ControlStatus::kConnecting));
    try {
      franka::Robot robot(robot_ip_);
      if (automatic_error_recovery_) {
        robot.automaticErrorRecovery();
      }

      bool motion_active = false;
      Eigen::Isometry3d command_transform = Eigen::Isometry3d::Identity();
      Target cached_target;
      bool cached_enabled = false;
      SteadyClock::time_point cached_enabled_stamp{};

      robot.control(
          [&](const franka::RobotState& robot_state,
              franka::Duration period) -> franka::CartesianPose {
            update_current_state(robot_state);
            const Eigen::Isometry3d current_transform = array_to_transform(robot_state.O_T_EE);

            if (!running_.load()) {
              status_.store(static_cast<int>(ControlStatus::kStopped));
              return franka::MotionFinished(franka::CartesianPose(transform_to_array(current_transform)));
            }

            const auto now = SteadyClock::now();
            bool enabled = false;
            {
              std::unique_lock<std::mutex> lock(target_mutex_, std::try_to_lock);
              if (lock.owns_lock()) {
                cached_target = target_;
                cached_enabled = teleop_enabled_;
                cached_enabled_stamp = enabled_stamp_;
              }
            }
            enabled =
                cached_enabled && seconds_since(cached_enabled_stamp, now) <= enabled_timeout_sec_;

            const bool target_fresh =
                cached_target.valid && seconds_since(cached_target.stamp, now) <= target_timeout_sec_;
            if (!enabled) {
              motion_active = false;
              command_transform = current_transform;
              status_.store(static_cast<int>(ControlStatus::kDisabled));
              return franka::CartesianPose(transform_to_array(command_transform));
            }

            if (!target_fresh) {
              motion_active = false;
              command_transform = current_transform;
              status_.store(static_cast<int>(ControlStatus::kWaitingForTarget));
              return franka::CartesianPose(transform_to_array(command_transform));
            }

            Eigen::Isometry3d target_transform = Eigen::Isometry3d::Identity();
            target_transform.translation() =
                clamp_position(cached_target.position, workspace_min_, workspace_max_);
            target_transform.linear() = cached_target.orientation.toRotationMatrix();

            if (!motion_active) {
              command_transform = current_transform;
              if (!initial_target_is_close_enough(current_transform, target_transform)) {
                status_.store(static_cast<int>(ControlStatus::kInitialTargetTooFar));
                return franka::CartesianPose(transform_to_array(command_transform));
              }
              motion_active = true;
            }

            const double dt = std::max(period.toSec(), 1e-4);
            command_transform = step_toward(command_transform, target_transform, dt);
            status_.store(static_cast<int>(ControlStatus::kRunning));
            return franka::CartesianPose(transform_to_array(command_transform));
          });
    } catch (const franka::Exception& exception) {
      status_.store(static_cast<int>(ControlStatus::kException));
      set_last_exception(exception.what());
      RCLCPP_ERROR(get_logger(), "libfranka control failed: %s", exception.what());
    } catch (const std::exception& exception) {
      status_.store(static_cast<int>(ControlStatus::kException));
      set_last_exception(exception.what());
      RCLCPP_ERROR(get_logger(), "Control thread failed: %s", exception.what());
    }
  }

  void update_current_state(const franka::RobotState& robot_state) {
    std::unique_lock<std::mutex> lock(current_state_mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {
      return;
    }
    current_state_.valid = true;
    current_state_.pose = robot_state.O_T_EE;
    current_state_.q = robot_state.q;
    current_state_.dq = robot_state.dq;
    current_state_.tau_j = robot_state.tau_J;
  }

  bool initial_target_is_close_enough(
      const Eigen::Isometry3d& current_transform,
      const Eigen::Isometry3d& target_transform) const {
    const double distance =
        (target_transform.translation() - current_transform.translation()).norm();
    if (max_initial_target_distance_m_ > 0.0 && distance > max_initial_target_distance_m_) {
      return false;
    }

    Eigen::Quaterniond current_q(current_transform.linear());
    Eigen::Quaterniond target_q(target_transform.linear());
    if (current_q.dot(target_q) < 0.0) {
      target_q.coeffs() *= -1.0;
    }
    const Eigen::AngleAxisd delta(current_q.inverse() * target_q);
    const double angle = std::abs(delta.angle());
    return !(max_initial_target_angle_rad_ > 0.0 && angle > max_initial_target_angle_rad_);
  }

  Eigen::Isometry3d step_toward(
      const Eigen::Isometry3d& current_transform,
      const Eigen::Isometry3d& target_transform,
      double dt) const {
    Eigen::Isometry3d next = Eigen::Isometry3d::Identity();

    Eigen::Vector3d delta_position =
        target_transform.translation() - current_transform.translation();
    const double distance = delta_position.norm();
    const double max_distance = std::max(max_linear_velocity_mps_ * dt, 0.0);
    if (distance > max_distance && max_distance > 0.0) {
      delta_position *= max_distance / distance;
    }
    next.translation() = current_transform.translation() + delta_position;

    Eigen::Quaterniond current_q(current_transform.linear());
    Eigen::Quaterniond target_q(target_transform.linear());
    if (current_q.dot(target_q) < 0.0) {
      target_q.coeffs() *= -1.0;
    }
    const Eigen::AngleAxisd delta_rotation(current_q.inverse() * target_q);
    const double angle = std::abs(delta_rotation.angle());
    const double max_angle = std::max(max_angular_velocity_radps_ * dt, 0.0);
    const double ratio = angle > 1e-9 ? std::min(1.0, max_angle / angle) : 1.0;
    Eigen::Quaterniond next_q = current_q.slerp(ratio, target_q).normalized();
    next.linear() = next_q.toRotationMatrix();
    return next;
  }

  void publish_state_timer() {
    CurrentState state;
    {
      std::lock_guard<std::mutex> lock(current_state_mutex_);
      state = current_state_;
    }

    if (state.valid) {
      publish_current_pose(state);
      publish_joint_state(state);
    }
    publish_debug(state.valid);
  }

  void publish_current_pose(const CurrentState& state) {
    const Eigen::Isometry3d transform = array_to_transform(state.pose);
    Eigen::Quaterniond orientation(transform.linear());
    orientation.normalize();

    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = base_frame_;
    msg.pose.position.x = transform.translation().x();
    msg.pose.position.y = transform.translation().y();
    msg.pose.position.z = transform.translation().z();
    msg.pose.orientation.x = orientation.x();
    msg.pose.orientation.y = orientation.y();
    msg.pose.orientation.z = orientation.z();
    msg.pose.orientation.w = orientation.w();
    current_pose_pub_->publish(msg);
  }

  void publish_joint_state(const CurrentState& state) {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name = joint_names_;
    msg.position.assign(state.q.begin(), state.q.end());
    msg.velocity.assign(state.dq.begin(), state.dq.end());
    msg.effort.assign(state.tau_j.begin(), state.tau_j.end());
    joint_state_pub_->publish(msg);
  }

  void publish_debug(bool have_current_state) {
    const auto status = static_cast<ControlStatus>(status_.load());
    std::ostringstream stream;
    stream << "{";
    stream << "\"status\":\"" << status_to_string(status) << "\",";
    stream << "\"robot_ip\":\"" << robot_ip_ << "\",";
    stream << "\"have_current_state\":" << (have_current_state ? "true" : "false") << ",";
    stream << "\"target_topic\":\"" << target_pose_topic_ << "\",";
    stream << "\"enabled_topic\":\"" << enabled_topic_ << "\",";
    stream << "\"base_frame\":\"" << base_frame_ << "\"";
    const auto exception = last_exception();
    if (!exception.empty()) {
      stream << ",\"exception\":\"" << exception << "\"";
    }
    stream << "}";

    std_msgs::msg::String msg;
    msg.data = stream.str();
    debug_pub_->publish(msg);
  }

  void set_last_exception(const std::string& exception) {
    std::lock_guard<std::mutex> lock(exception_mutex_);
    last_exception_ = exception;
  }

  std::string last_exception() const {
    std::lock_guard<std::mutex> lock(exception_mutex_);
    return last_exception_;
  }

  std::string robot_ip_;
  std::string target_pose_topic_;
  std::string enabled_topic_;
  std::string current_pose_topic_;
  std::string debug_topic_;
  std::string base_frame_;
  double publish_rate_hz_{50.0};
  double target_timeout_sec_{0.25};
  double enabled_timeout_sec_{0.25};
  double max_linear_velocity_mps_{0.04};
  double max_angular_velocity_radps_{0.25};
  double max_initial_target_distance_m_{0.08};
  double max_initial_target_angle_rad_{0.6};
  Eigen::Vector3d workspace_min_{0.20, -0.45, 0.08};
  Eigen::Vector3d workspace_max_{0.80, 0.45, 0.75};
  std::vector<std::string> joint_names_;
  bool automatic_error_recovery_{true};

  std::mutex target_mutex_;
  Target target_;
  bool teleop_enabled_{false};
  SteadyClock::time_point enabled_stamp_{};

  std::mutex current_state_mutex_;
  CurrentState current_state_;

  mutable std::mutex exception_mutex_;
  std::string last_exception_;

  std::atomic<bool> running_{false};
  std::atomic<int> status_{static_cast<int>(ControlStatus::kConnecting)};
  std::thread control_thread_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enabled_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LibfrankaCartesianPoseNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
