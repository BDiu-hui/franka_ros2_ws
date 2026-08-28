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
    auto_declare<std::vector<std::string>>(
        "joint_names", {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
                        "panda_joint5", "panda_joint6", "panda_joint7"});

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
    RCLCPP_ERROR(get_node()->get_logger(), "Expected %zu joint names, got %zu", kNumJoints,
                 joint_names_.size());
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
  compliance_params_.rotational_clip_x = get_node()->get_parameter("rotational_clip_x").as_double();
  compliance_params_.rotational_clip_y = get_node()->get_parameter("rotational_clip_y").as_double();
  compliance_params_.rotational_clip_z = get_node()->get_parameter("rotational_clip_z").as_double();
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
  realtime_jacobian_publisher_ = std::make_shared<
      realtime_tools::RealtimePublisher<serl_franka_controllers_ros2::msg::ZeroJacobian>>(
      jacobian_publisher_);

  equilibrium_pose_subscriber_ =
      get_node()->create_subscription<serl_franka_controllers_ros2::msg::CartesianImpedanceCommand>(
          "~/equilibrium_pose", rclcpp::SystemDefaultsQoS(),
          std::bind(&CartesianImpedanceController::equilibrium_pose_callback, this,
                    std::placeholders::_1));

  parameter_callback_handle_ = get_node()->add_on_set_parameters_callback(
      std::bind(&CartesianImpedanceController::on_parameters_set, this, std::placeholders::_1));

  compliance_params_buffer_.writeFromNonRT(compliance_params_);
  target_command_buffer_.writeFromNonRT(ImpedanceTarget{});

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
  const Eigen::Isometry3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
  const Eigen::Map<const Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());

  ImpedanceTarget initial_target;
  initial_target.position = initial_transform.translation();
  initial_target.orientation = Eigen::Quaterniond(initial_transform.linear());
  initial_target.master_q = q_initial;
  target_command_buffer_.writeFromNonRT(initial_target);

  ImpedanceInput initial_input;
  initial_input.q = q_initial;
  initial_input.ee_pose = initial_transform;
  impedance_core_.reset(initial_input, initial_target, *compliance_params_buffer_.readFromRT());
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

  ImpedanceInput input;
  input.coriolis = Eigen::Map<const Vector7d>(coriolis_array.data());
  input.jacobian = Eigen::Map<const Matrix67d>(jacobian_array_.data());
  input.q = Eigen::Map<const Vector7d>(robot_state.q.data());
  input.dq = Eigen::Map<const Vector7d>(robot_state.dq.data());
  input.tau_j_d = Eigen::Map<const Vector7d>(robot_state.tau_J_d.data());
  input.ee_pose = Eigen::Isometry3d(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));

  const ImpedanceTarget target = *target_command_buffer_.readFromRT();
  const ComplianceParams params = *compliance_params_buffer_.readFromRT();
  std::optional<ElbowInput> elbow;
  if (impedance_core_.needsElbowInput(target, params)) {
    const auto elbow_pose_array = franka_robot_model_->getPoseMatrix(franka::Frame::kJoint4);
    const Eigen::Affine3d elbow_transform(Eigen::Matrix4d::Map(elbow_pose_array.data()));
    const auto elbow_jac_array = franka_robot_model_->getZeroJacobian(franka::Frame::kJoint4);
    elbow.emplace();
    elbow->position = elbow_transform.translation();
    elbow->jacobian = Eigen::Map<const Matrix67d>(elbow_jac_array.data()).topRows(3);
  }

  const Vector7d torque = impedance_core_.update(input, target, params, elbow);

  for (size_t i = 0; i < kNumJoints; ++i) {
    command_interfaces_[i].set_value(torque(static_cast<Eigen::Index>(i)));
  }

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

void CartesianImpedanceController::equilibrium_pose_callback(
    const serl_franka_controllers_ros2::msg::CartesianImpedanceCommand::SharedPtr msg) {
  ImpedanceTarget target = *target_command_buffer_.readFromNonRT();
  target.position << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  const Eigen::Quaterniond previous_orientation(target.orientation);
  target.orientation.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  if (previous_orientation.coeffs().dot(target.orientation.coeffs()) < 0.0) {
    target.orientation.coeffs() = -target.orientation.coeffs();
  }
  if (msg->has_master_q) {
    for (size_t i = 0; i < kNumJoints; ++i) {
      target.master_q(static_cast<Eigen::Index>(i)) = msg->master_q[i];
    }
    target.has_master_q = true;
  } else {
    target.has_master_q = false;
  }
  ++target.sequence;
  target_command_buffer_.writeFromNonRT(target);
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
  compliance_params_buffer_.writeFromNonRT(compliance_params_);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

franka::RobotState* CartesianImpedanceController::get_robot_state_ptr() {
  const auto state_interface_name = hardware_prefix_ + "/robot_state";
  auto state_it = std::find_if(state_interfaces_.begin(), state_interfaces_.end(),
                               [&](const auto& state_interface) {
                                 return state_interface.get_name() == state_interface_name;
                               });
  if (state_it == state_interfaces_.end()) {
    return nullptr;
  }
  return bit_cast<franka::RobotState*>(state_it->get_value());
}

}  // namespace serl_franka_controllers

PLUGINLIB_EXPORT_CLASS(serl_franka_controllers::CartesianImpedanceController,
                       controller_interface::ControllerInterface)
