/*
Reference:
  https://github.com/frankarobotics/franka_ros2
*/

#include <serl_franka_controllers_ros2/cartesian_impedance_controller.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>
#include <exception>
#include <functional>
#include <string>
#include <utility>

#include <pluginlib/class_list_macros.hpp>

namespace {

template <class To, class From>
std::enable_if_t<sizeof(To) == sizeof(From) && std::is_trivially_copyable_v<From> &&
                     std::is_trivially_copyable_v<To>,
                 To>
bit_cast(const From& src) noexcept {
  static_assert(std::is_trivially_constructible_v<To>,
                "This implementation requires a trivially constructible destination type");
  To dst;
  std::memcpy(&dst, &src, sizeof(To));
  return dst;
}

}  // namespace

namespace serl_franka_controllers {

controller_interface::InterfaceConfiguration
CartesianImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint_name : joint_names_) {
    config.names.push_back(joint_name + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
CartesianImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint_name : joint_names_) {
    config.names.push_back(joint_name + "/position");
    config.names.push_back(joint_name + "/velocity");
  }
  for (const auto& state_interface_name : franka_robot_model_->get_state_interface_names()) {
    config.names.push_back(state_interface_name);
  }
  return config;
}

CallbackReturn CartesianImpedanceController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "panda");
    auto_declare<std::string>("robot_type", "");
    auto_declare<std::string>("arm_prefix", "");
    auto_declare<std::vector<std::string>>("joint_names",
                                           {"panda_joint1", "panda_joint2", "panda_joint3",
                                            "panda_joint4", "panda_joint5", "panda_joint6",
                                            "panda_joint7"});

    auto_declare<double>("translational_stiffness", compliance_params_.translational_stiffness);
    auto_declare<double>("translational_damping", compliance_params_.translational_damping);
    auto_declare<double>("rotational_stiffness", compliance_params_.rotational_stiffness);
    auto_declare<double>("rotational_damping", compliance_params_.rotational_damping);
    auto_declare<double>("nullspace_stiffness", compliance_params_.nullspace_stiffness);
    auto_declare<double>("joint1_nullspace_stiffness",
                         compliance_params_.joint1_nullspace_stiffness);
    auto_declare<double>("translational_clip_neg_x", compliance_params_.translational_clip_neg_x);
    auto_declare<double>("translational_clip_neg_y", compliance_params_.translational_clip_neg_y);
    auto_declare<double>("translational_clip_neg_z", compliance_params_.translational_clip_neg_z);
    auto_declare<double>("translational_clip_x", compliance_params_.translational_clip_x);
    auto_declare<double>("translational_clip_y", compliance_params_.translational_clip_y);
    auto_declare<double>("translational_clip_z", compliance_params_.translational_clip_z);
    auto_declare<double>("rotational_clip_neg_x", compliance_params_.rotational_clip_neg_x);
    auto_declare<double>("rotational_clip_neg_y", compliance_params_.rotational_clip_neg_y);
    auto_declare<double>("rotational_clip_neg_z", compliance_params_.rotational_clip_neg_z);
    auto_declare<double>("rotational_clip_x", compliance_params_.rotational_clip_x);
    auto_declare<double>("rotational_clip_y", compliance_params_.rotational_clip_y);
    auto_declare<double>("rotational_clip_z", compliance_params_.rotational_clip_z);
    auto_declare<double>("translational_ki", compliance_params_.translational_ki);
    auto_declare<double>("rotational_ki", compliance_params_.rotational_ki);
    auto_declare<double>("filter_params", compliance_params_.filter_params);
    auto_declare<double>("elbow_stiffness", compliance_params_.elbow_stiffness);
    auto_declare<double>("elbow_damping", compliance_params_.elbow_damping);
    auto_declare<double>("jacobian_publish_rate", 100.0);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Init failed: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

bool CartesianImpedanceController::read_parameters() {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  robot_type_ = get_node()->get_parameter("robot_type").as_string();
  arm_prefix_ = get_node()->get_parameter("arm_prefix").as_string();
  joint_names_ = get_node()->get_parameter("joint_names").as_string_array();

  if (joint_names_.size() != kNumJoints) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Expected %zu joint names, got %zu", kNumJoints, joint_names_.size());
    return false;
  }

  if (robot_type_.empty()) {
    robot_type_ = arm_id_;
  }
  if (!arm_prefix_.empty()) {
    hardware_prefix_ = arm_prefix_ + "_" + robot_type_;
  } else {
    hardware_prefix_ = robot_type_;
  }

  compliance_params_.translational_stiffness =
      get_node()->get_parameter("translational_stiffness").as_double();
  compliance_params_.translational_damping =
      get_node()->get_parameter("translational_damping").as_double();
  compliance_params_.rotational_stiffness =
      get_node()->get_parameter("rotational_stiffness").as_double();
  compliance_params_.rotational_damping =
      get_node()->get_parameter("rotational_damping").as_double();
  compliance_params_.nullspace_stiffness =
      get_node()->get_parameter("nullspace_stiffness").as_double();
  compliance_params_.joint1_nullspace_stiffness =
      get_node()->get_parameter("joint1_nullspace_stiffness").as_double();
  compliance_params_.translational_clip_neg_x =
      get_node()->get_parameter("translational_clip_neg_x").as_double();
  compliance_params_.translational_clip_neg_y =
      get_node()->get_parameter("translational_clip_neg_y").as_double();
  compliance_params_.translational_clip_neg_z =
      get_node()->get_parameter("translational_clip_neg_z").as_double();
  compliance_params_.translational_clip_x =
      get_node()->get_parameter("translational_clip_x").as_double();
  compliance_params_.translational_clip_y =
      get_node()->get_parameter("translational_clip_y").as_double();
  compliance_params_.translational_clip_z =
      get_node()->get_parameter("translational_clip_z").as_double();
  compliance_params_.rotational_clip_neg_x =
      get_node()->get_parameter("rotational_clip_neg_x").as_double();
  compliance_params_.rotational_clip_neg_y =
      get_node()->get_parameter("rotational_clip_neg_y").as_double();
  compliance_params_.rotational_clip_neg_z =
      get_node()->get_parameter("rotational_clip_neg_z").as_double();
  compliance_params_.rotational_clip_x =
      get_node()->get_parameter("rotational_clip_x").as_double();
  compliance_params_.rotational_clip_y =
      get_node()->get_parameter("rotational_clip_y").as_double();
  compliance_params_.rotational_clip_z =
      get_node()->get_parameter("rotational_clip_z").as_double();
  compliance_params_.translational_ki = get_node()->get_parameter("translational_ki").as_double();
  compliance_params_.rotational_ki = get_node()->get_parameter("rotational_ki").as_double();
  compliance_params_.filter_params = get_node()->get_parameter("filter_params").as_double();
  compliance_params_.elbow_stiffness = get_node()->get_parameter("elbow_stiffness").as_double();
  compliance_params_.elbow_damping = get_node()->get_parameter("elbow_damping").as_double();
  set_jacobian_publish_rate(get_node()->get_parameter("jacobian_publish_rate").as_double());

  return true;
}

CallbackReturn CartesianImpedanceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (!read_parameters()) {
    return CallbackReturn::ERROR;
  }

  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      hardware_prefix_ + "/robot_model", hardware_prefix_ + "/robot_state");

  jacobian_publisher_ =
      get_node()->create_publisher<serl_franka_controllers_ros2::msg::ZeroJacobian>(
          "~/franka_jacobian", rclcpp::SystemDefaultsQoS());
  realtime_jacobian_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<
          serl_franka_controllers_ros2::msg::ZeroJacobian>>(jacobian_publisher_);

  equilibrium_pose_subscriber_ =
      get_node()->create_subscription<serl_franka_controllers_ros2::msg::CartesianImpedanceCommand>(
      "~/equilibrium_pose", rclcpp::SystemDefaultsQoS(),
      std::bind(&CartesianImpedanceController::equilibrium_pose_callback, this,
                std::placeholders::_1));

  parameter_callback_handle_ = get_node()->add_on_set_parameters_callback(
      std::bind(&CartesianImpedanceController::on_parameters_set, this, std::placeholders::_1));

  apply_compliance_params(compliance_params_);
  position_d_.setZero();
  position_d_target_.setZero();
  orientation_d_ = Eigen::Quaterniond::Identity();
  orientation_d_target_ = Eigen::Quaterniond::Identity();
  error_i_.setZero();

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
  robot_state_ptr_ = get_robot_state_ptr();
  if (robot_state_ptr_ == nullptr) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to obtain Franka robot_state interface");
    return CallbackReturn::ERROR;
  }

  const auto initial_state = *robot_state_ptr_;
  const auto zero_jacobian = franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
  jacobian_array_ = zero_jacobian;
  const Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
  const Eigen::Map<const Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());

  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear());

  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    position_d_target_ = position_d_;
    orientation_d_target_ = orientation_d_;
  }

  q_d_nullspace_ = q_initial;
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    q_master_target_ = q_initial;
    have_master_q_ = false;
  }
  q_master_ = q_initial;
  error_i_.setZero();
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->release_interfaces();
  robot_state_ptr_ = nullptr;
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianImpedanceController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  if (robot_state_ptr_ == nullptr) {
    return controller_interface::return_type::ERROR;
  }

  const auto& robot_state = *robot_state_ptr_;
  const auto coriolis_array = franka_robot_model_->getCoriolisForceVector();
  jacobian_array_ = franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
  if (jacobian_publish_decimation_ > 0 &&
      ++jacobian_publish_counter_ >= jacobian_publish_decimation_) {
    jacobian_publish_counter_ = 0;
    publish_zero_jacobian();
  }

  const Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  const Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array_.data());
  const Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  const Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  const Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_j_d(robot_state.tau_J_d.data());
  const Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());

  Eigen::Vector3d position_target;
  Eigen::Quaterniond orientation_target;
  Eigen::Matrix<double, 7, 1> q_master_target_local;
  bool have_master_q_local;
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    position_target = position_d_target_;
    orientation_target = orientation_d_target_;
    q_master_target_local = q_master_target_;
    have_master_q_local = have_master_q_;
  }

  error_.head(3) = position - position_d_;
  for (int i = 0; i < 3; ++i) {
    error_(i) = std::min(std::max(error_(i), translational_clip_min_(i)), translational_clip_max_(i));
  }

  if (orientation_target.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() = -orientation.coeffs();
  }
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error_.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  error_.tail(3) = -transform.linear() * error_.tail(3);
  for (int i = 0; i < 3; ++i) {
    error_(i + 3) =
        std::min(std::max(error_(i + 3), rotational_clip_min_(i)), rotational_clip_max_(i));
  }

  error_i_.head(3) = (error_i_.head(3) + error_.head(3)).cwiseMax(-0.1).cwiseMin(0.1);
  error_i_.tail(3) = (error_i_.tail(3) + error_.tail(3)).cwiseMax(-0.3).cwiseMin(0.3);

  Eigen::Matrix<double, 7, 1> tau_task;
  Eigen::Matrix<double, 7, 1> tau_nullspace;
  Eigen::Matrix<double, 7, 1> tau_d;
  const Eigen::Matrix<double, 6, 6> jj_t_damped =
      jacobian * jacobian.transpose() +
      (kDampedPseudoInverseLambda * kDampedPseudoInverseLambda) *
          Eigen::Matrix<double, 6, 6>::Identity();
  const Eigen::Matrix<double, 6, 7> jacobian_transpose_pinv =
      jj_t_damped.ldlt().solve(jacobian);
  const Eigen::Matrix<double, 7, 7> nullspace_projector =
      Eigen::Matrix<double, 7, 7>::Identity() -
      jacobian.transpose() * jacobian_transpose_pinv;

  tau_task = jacobian.transpose() *
             (-cartesian_stiffness_ * error_ - cartesian_damping_ * (jacobian * dq) -
              ki_ * error_i_);

  Eigen::Matrix<double, 7, 1> qe = q_d_nullspace_ - q;
  qe.head(1) *= joint1_nullspace_stiffness_;
  Eigen::Matrix<double, 7, 1> dqe = dq;
  dqe.head(1) *= 2.0 * std::sqrt(joint1_nullspace_stiffness_);

  tau_nullspace =
      nullspace_projector *
      (nullspace_stiffness_ * qe - (2.0 * std::sqrt(nullspace_stiffness_)) * dqe);

  Eigen::Matrix<double, 7, 1> tau_elbow_null = Eigen::Matrix<double, 7, 1>::Zero();
  if (have_master_q_local && elbow_stiffness_ > 0.0) {
    const Eigen::Vector3d elbow_master = compute_elbow_position(q_master_);
    const auto elbow_pose_array = franka_robot_model_->getPoseMatrix(franka::Frame::kJoint4);
    const Eigen::Affine3d elbow_transform(Eigen::Matrix4d::Map(elbow_pose_array.data()));
    const Eigen::Vector3d elbow_slave = elbow_transform.translation();
    const auto elbow_jac_array = franka_robot_model_->getZeroJacobian(franka::Frame::kJoint4);
    const Eigen::Map<const Eigen::Matrix<double, 6, 7>> J_e_full(elbow_jac_array.data());
    const Eigen::Matrix<double, 3, 7> J_e = J_e_full.topRows(3);

    const Eigen::Vector3d e_pos = elbow_master - elbow_slave;
    const Eigen::Matrix<double, 7, 1> tau_elbow_task =
        J_e.transpose() * (elbow_stiffness_ * e_pos - elbow_damping_ * (J_e * dq));

    tau_elbow_null = nullspace_projector * tau_elbow_task;
  }

  tau_d = tau_task + tau_nullspace + tau_elbow_null + coriolis;
  tau_d = saturate_torque_rate(tau_d, tau_j_d);

  for (size_t i = 0; i < kNumJoints; ++i) {
    command_interfaces_[i].set_value(tau_d(static_cast<Eigen::Index>(i)));
  }

  cartesian_stiffness_ = compliance_params_.filter_params * cartesian_stiffness_target_ +
                         (1.0 - compliance_params_.filter_params) * cartesian_stiffness_;
  cartesian_damping_ = compliance_params_.filter_params * cartesian_damping_target_ +
                       (1.0 - compliance_params_.filter_params) * cartesian_damping_;
  nullspace_stiffness_ = compliance_params_.filter_params * nullspace_stiffness_target_ +
                         (1.0 - compliance_params_.filter_params) * nullspace_stiffness_;
  joint1_nullspace_stiffness_ =
      compliance_params_.filter_params * joint1_nullspace_stiffness_target_ +
      (1.0 - compliance_params_.filter_params) * joint1_nullspace_stiffness_;
  position_d_ = compliance_params_.filter_params * position_target +
                (1.0 - compliance_params_.filter_params) * position_d_;
  orientation_d_ = orientation_d_.slerp(compliance_params_.filter_params, orientation_target);
  ki_ = compliance_params_.filter_params * ki_target_ +
        (1.0 - compliance_params_.filter_params) * ki_;
  q_master_ = compliance_params_.filter_params * q_master_target_local +
              (1.0 - compliance_params_.filter_params) * q_master_;
  elbow_stiffness_ = compliance_params_.filter_params * elbow_stiffness_target_ +
                     (1.0 - compliance_params_.filter_params) * elbow_stiffness_;
  elbow_damping_ = compliance_params_.filter_params * elbow_damping_target_ +
                   (1.0 - compliance_params_.filter_params) * elbow_damping_;

  return controller_interface::return_type::OK;
}

void CartesianImpedanceController::publish_zero_jacobian() {
  if (!realtime_jacobian_publisher_ || !realtime_jacobian_publisher_->trylock()) {
    return;
  }
  for (size_t i = 0; i < jacobian_array_.size(); ++i) {
    realtime_jacobian_publisher_->msg_.zero_jacobian[i] = jacobian_array_[i];
  }
  realtime_jacobian_publisher_->unlockAndPublish();
}

void CartesianImpedanceController::set_jacobian_publish_rate(double publish_rate_hz) {
  if (publish_rate_hz <= 0.0) {
    jacobian_publish_decimation_ = 0;
    jacobian_publish_counter_ = 0;
    return;
  }

  constexpr double kControllerUpdateRateHz = 1000.0;
  jacobian_publish_decimation_ =
      std::max(1, static_cast<int>(std::lround(kControllerUpdateRateHz / publish_rate_hz)));
  jacobian_publish_counter_ = 0;
}

Eigen::Matrix<double, 7, 1> CartesianImpedanceController::saturate_torque_rate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_j_d) const {
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < kNumJoints; ++i) {
    const double difference =
        tau_d_calculated(static_cast<Eigen::Index>(i)) - tau_j_d(static_cast<Eigen::Index>(i));
    tau_d_saturated(static_cast<Eigen::Index>(i)) =
        tau_j_d(static_cast<Eigen::Index>(i)) +
        std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}

void CartesianImpedanceController::apply_compliance_params(const ComplianceParams& params) {
  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_.topLeftCorner(3, 3) =
      params.translational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_stiffness_target_.bottomRightCorner(3, 3) =
      params.rotational_stiffness * Eigen::Matrix3d::Identity();

  cartesian_damping_target_.setIdentity();
  cartesian_damping_target_.topLeftCorner(3, 3) =
      params.translational_damping * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.bottomRightCorner(3, 3) =
      params.rotational_damping * Eigen::Matrix3d::Identity();

  nullspace_stiffness_target_ = params.nullspace_stiffness;
  joint1_nullspace_stiffness_target_ = params.joint1_nullspace_stiffness;
  elbow_stiffness_target_ = params.elbow_stiffness;
  elbow_damping_target_ = params.elbow_damping;

  translational_clip_min_ << -params.translational_clip_neg_x, -params.translational_clip_neg_y,
      -params.translational_clip_neg_z;
  translational_clip_max_ << params.translational_clip_x, params.translational_clip_y,
      params.translational_clip_z;
  rotational_clip_min_ << -params.rotational_clip_neg_x, -params.rotational_clip_neg_y,
      -params.rotational_clip_neg_z;
  rotational_clip_max_ << params.rotational_clip_x, params.rotational_clip_y,
      params.rotational_clip_z;

  ki_target_.setIdentity();
  ki_target_.topLeftCorner(3, 3) = params.translational_ki * Eigen::Matrix3d::Identity();
  ki_target_.bottomRightCorner(3, 3) = params.rotational_ki * Eigen::Matrix3d::Identity();

  cartesian_stiffness_ = cartesian_stiffness_target_;
  cartesian_damping_ = cartesian_damping_target_;
  ki_ = ki_target_;
  nullspace_stiffness_ = nullspace_stiffness_target_;
  joint1_nullspace_stiffness_ = joint1_nullspace_stiffness_target_;
  elbow_stiffness_ = elbow_stiffness_target_;
  elbow_damping_ = elbow_damping_target_;
}

void CartesianImpedanceController::equilibrium_pose_callback(
    const serl_franka_controllers_ros2::msg::CartesianImpedanceCommand::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(target_mutex_);
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  error_i_.setZero();
  const Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() = -orientation_d_target_.coeffs();
  }
  if (msg->has_master_q) {
    for (size_t i = 0; i < kNumJoints; ++i) {
      q_master_target_(static_cast<Eigen::Index>(i)) = msg->master_q[i];
    }
    have_master_q_ = true;
  } else {
    have_master_q_ = false;
  }
}

Eigen::Vector3d CartesianImpedanceController::compute_elbow_position(
    const Eigen::Matrix<double, 7, 1>& q) const {
  // Modified-DH parameters for Franka FR3 / Panda (frames 0 through 4).
  // a_{i-1}, alpha_{i-1}, d_i for i = 1..4 (joint 4 origin in base frame).
  static constexpr std::array<double, 4> a_prev = {0.0, 0.0, 0.0, 0.0825};
  static constexpr std::array<double, 4> alpha_prev = {0.0, -M_PI_2, M_PI_2, M_PI_2};
  static constexpr std::array<double, 4> d = {0.333, 0.0, 0.316, 0.0};

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  for (int i = 0; i < 4; ++i) {
    const double ca = std::cos(alpha_prev[i]);
    const double sa = std::sin(alpha_prev[i]);
    const double ct = std::cos(q(i));
    const double st = std::sin(q(i));
    Eigen::Matrix4d A;
    A << ct,     -st,     0.0,   a_prev[i],
         st * ca, ct * ca, -sa,  -sa * d[i],
         st * sa, ct * sa,  ca,   ca * d[i],
         0.0,    0.0,     0.0,   1.0;
    T = T * A;
  }
  return T.block<3, 1>(0, 3);
}

rcl_interfaces::msg::SetParametersResult CartesianImpedanceController::on_parameters_set(
    const std::vector<rclcpp::Parameter>& parameters) {
  auto updated = compliance_params_;
  for (const auto& parameter : parameters) {
    const auto& name = parameter.get_name();
    if (name == "translational_stiffness") {
      updated.translational_stiffness = parameter.as_double();
    } else if (name == "translational_damping") {
      updated.translational_damping = parameter.as_double();
    } else if (name == "rotational_stiffness") {
      updated.rotational_stiffness = parameter.as_double();
    } else if (name == "rotational_damping") {
      updated.rotational_damping = parameter.as_double();
    } else if (name == "nullspace_stiffness") {
      updated.nullspace_stiffness = parameter.as_double();
    } else if (name == "joint1_nullspace_stiffness") {
      updated.joint1_nullspace_stiffness = parameter.as_double();
    } else if (name == "translational_clip_neg_x") {
      updated.translational_clip_neg_x = parameter.as_double();
    } else if (name == "translational_clip_neg_y") {
      updated.translational_clip_neg_y = parameter.as_double();
    } else if (name == "translational_clip_neg_z") {
      updated.translational_clip_neg_z = parameter.as_double();
    } else if (name == "translational_clip_x") {
      updated.translational_clip_x = parameter.as_double();
    } else if (name == "translational_clip_y") {
      updated.translational_clip_y = parameter.as_double();
    } else if (name == "translational_clip_z") {
      updated.translational_clip_z = parameter.as_double();
    } else if (name == "rotational_clip_neg_x") {
      updated.rotational_clip_neg_x = parameter.as_double();
    } else if (name == "rotational_clip_neg_y") {
      updated.rotational_clip_neg_y = parameter.as_double();
    } else if (name == "rotational_clip_neg_z") {
      updated.rotational_clip_neg_z = parameter.as_double();
    } else if (name == "rotational_clip_x") {
      updated.rotational_clip_x = parameter.as_double();
    } else if (name == "rotational_clip_y") {
      updated.rotational_clip_y = parameter.as_double();
    } else if (name == "rotational_clip_z") {
      updated.rotational_clip_z = parameter.as_double();
    } else if (name == "translational_ki") {
      updated.translational_ki = parameter.as_double();
    } else if (name == "rotational_ki") {
      updated.rotational_ki = parameter.as_double();
    } else if (name == "filter_params") {
      updated.filter_params = parameter.as_double();
    } else if (name == "elbow_stiffness") {
      updated.elbow_stiffness = parameter.as_double();
    } else if (name == "elbow_damping") {
      updated.elbow_damping = parameter.as_double();
    } else if (name == "jacobian_publish_rate") {
      set_jacobian_publish_rate(parameter.as_double());
    }
  }

  compliance_params_ = updated;
  apply_compliance_params(compliance_params_);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

franka::RobotState* CartesianImpedanceController::get_robot_state_ptr() {
  const auto state_interface_name = hardware_prefix_ + "/robot_state";
  auto state_it = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(),
      [&](const auto& state_interface) { return state_interface.get_name() == state_interface_name; });
  if (state_it == state_interfaces_.end()) {
    return nullptr;
  }
  return bit_cast<franka::RobotState*>(state_it->get_value());
}

}  // namespace serl_franka_controllers

PLUGINLIB_EXPORT_CLASS(serl_franka_controllers::CartesianImpedanceController,
                       controller_interface::ControllerInterface)
