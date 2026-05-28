#include <serl_franka_controllers_ros2/impedance_tuning_panel.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <sstream>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/contexts/default_context.hpp>
#include <rclcpp/parameter.hpp>

namespace serl_franka_controllers_ros2 {

namespace {

using namespace std::chrono_literals;

std::string format_value(double value) {
  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream.precision(3);
  stream << value;
  return stream.str();
}

std::string yaml_value(double value) {
  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream.precision(6);
  stream << value;
  return stream.str();
}

}  // namespace

ImpedanceTuningPanel::ImpedanceTuningPanel(QWidget* parent) : rviz_common::Panel(parent) {
  build_ui();
  setup_ros();
}

ImpedanceTuningPanel::~ImpedanceTuningPanel() {
  if (executor_ && node_) {
    executor_->remove_node(node_);
  }
}

void ImpedanceTuningPanel::onInitialize() {
  rviz_common::Panel::onInitialize();
  request_current_parameters();
}

void ImpedanceTuningPanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
  config.mapSetValue("target_node", target_node_edit_->text());
}

void ImpedanceTuningPanel::load(const rviz_common::Config& config) {
  rviz_common::Panel::load(config);
  QString target_node;
  if (config.mapGetString("target_node", &target_node)) {
    target_node_edit_->setText(target_node);
    on_target_node_changed();
  }
}

void ImpedanceTuningPanel::setup_ros() {
  if (!rclcpp::contexts::get_global_default_context()->is_valid()) {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  node_ = std::make_shared<rclcpp::Node>("serl_impedance_tuning_panel");
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);

  parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
      node_, target_node_edit_->text().toStdString());

  spin_timer_ = new QTimer(this);
  connect(spin_timer_, &QTimer::timeout, this, [this]() {
    if (executor_) {
      executor_->spin_some();
    }
  });
  spin_timer_->start(50);
}

void ImpedanceTuningPanel::build_ui() {
  auto* root_layout = new QVBoxLayout;

  auto* target_layout = new QHBoxLayout;
  auto* target_label = new QLabel("Controller Node");
  target_node_edit_ = new QLineEdit("/cartesian_impedance_controller");
  refresh_button_ = new QPushButton("Refresh");
  raw_config_button_ = new QPushButton("Raw Config");
  set_defaults_button_ = new QPushButton("Set Defaults");
  target_layout->addWidget(target_label);
  target_layout->addWidget(target_node_edit_);
  target_layout->addWidget(refresh_button_);
  target_layout->addWidget(raw_config_button_);
  target_layout->addWidget(set_defaults_button_);
  root_layout->addLayout(target_layout);

  status_label_ = new QLabel("Ready");
  root_layout->addWidget(status_label_);

  const std::vector<SliderSpec> specs = {
      {"translational_stiffness", 50.0, 4000.0, 10.0, 2000.0, "Translational Stiffness"},
      {"translational_damping", 1.0, 150.0, 1.0, 89.0, "Translational Damping"},
      {"rotational_stiffness", 10.0, 400.0, 1.0, 300.0, "Rotational Stiffness"}, //150
      {"rotational_damping", 1.0, 40.0, 0.5, 7.0, "Rotational Damping"},
      {"nullspace_stiffness", 0.0, 50.0, 0.1, 0.2, "Nullspace Stiffness"},
      {"joint1_nullspace_stiffness", 0.0, 300.0, 1.0, 100.0, "Joint1 Nullspace"},
      {"translational_clip_x", 0.001, 0.10, 0.001, 0.01, "Clip +X"},
      {"translational_clip_y", 0.001, 0.10, 0.001, 0.01, "Clip +Y"},
      {"translational_clip_z", 0.001, 0.10, 0.001, 0.01, "Clip +Z"},
      {"translational_clip_neg_x", 0.001, 0.10, 0.001, 0.01, "Clip -X"},
      {"translational_clip_neg_y", 0.001, 0.10, 0.001, 0.01, "Clip -Y"},
      {"translational_clip_neg_z", 0.001, 0.10, 0.001, 0.01, "Clip -Z"},
      {"rotational_clip_x", 0.001, 0.30, 0.001, 0.05, "Rot Clip +X"},
      {"rotational_clip_y", 0.001, 0.30, 0.001, 0.05, "Rot Clip +Y"},
      {"rotational_clip_z", 0.001, 0.30, 0.001, 0.05, "Rot Clip +Z"},
      {"rotational_clip_neg_x", 0.001, 0.30, 0.001, 0.05, "Rot Clip -X"},
      {"rotational_clip_neg_y", 0.001, 0.30, 0.001, 0.05, "Rot Clip -Y"},
      {"rotational_clip_neg_z", 0.001, 0.30, 0.001, 0.05, "Rot Clip -Z"},
      {"translational_ki", 0.0, 50.0, 0.1, 0.0, "Translational KI"},
      {"rotational_ki", 0.0, 50.0, 0.1, 0.0, "Rotational KI"},
      {"filter_params", 0.001, 0.10, 0.001, 0.005, "Filter Params"},
  };

  for (const auto& spec : specs) {
    add_slider(root_layout, spec);
  }

  root_layout->addStretch();
  setLayout(root_layout);

  connect(refresh_button_, &QPushButton::clicked, this, &ImpedanceTuningPanel::on_refresh_clicked);
  connect(raw_config_button_, &QPushButton::clicked, this,
          &ImpedanceTuningPanel::on_raw_config_clicked);
  connect(set_defaults_button_, &QPushButton::clicked, this,
          &ImpedanceTuningPanel::on_set_defaults_clicked);
  connect(target_node_edit_, &QLineEdit::editingFinished, this,
          &ImpedanceTuningPanel::on_target_node_changed);
}

void ImpedanceTuningPanel::add_slider(QVBoxLayout* layout, const SliderSpec& spec) {
  auto* row = new QHBoxLayout;
  auto* label = new QLabel(QString::fromStdString(spec.label));
  auto* slider = new QSlider(Qt::Horizontal);
  auto* value_label = new QLabel(QString::fromStdString(format_value(spec.default_value)));

  const int slider_max = static_cast<int>(std::llround((spec.max_value - spec.min_value) / spec.step));
  slider->setMinimum(0);
  slider->setMaximum(slider_max);
  slider->setValue(to_slider_units(spec, spec.default_value));

  row->addWidget(label);
  row->addWidget(slider, 1);
  row->addWidget(value_label);
  layout->addLayout(row);

  sliders_.emplace(spec.parameter_name, SliderWidgets{spec, value_label, slider});

  connect(slider, &QSlider::valueChanged, this, [this, spec, value_label](int slider_value) {
    const double value = from_slider_units(spec, slider_value);
    value_label->setText(QString::fromStdString(format_value(value)));
    if (!updating_from_remote_) {
      set_parameter_from_slider(spec, slider_value);
    }
  });
}

void ImpedanceTuningPanel::request_current_parameters() {
  if (!parameters_client_) {
    return;
  }

  if (!parameters_client_->wait_for_service(1s)) {
    status_label_->setText("Parameter service unavailable");
    return;
  }

  std::vector<std::string> names;
  names.reserve(sliders_.size());
  for (const auto& [name, widgets] : sliders_) {
    (void)widgets;
    names.push_back(name);
  }

  auto future = parameters_client_->get_parameters(names);
  if (executor_->spin_until_future_complete(future, 2s) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    status_label_->setText("Timed out while loading parameters");
    return;
  }
  const auto result = future.get();

  updating_from_remote_ = true;
  for (const auto& parameter : result) {
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      update_slider_display(parameter.get_name(), parameter.as_double());
    }
  }
  updating_from_remote_ = false;
  status_label_->setText("Loaded current controller parameters");
}

void ImpedanceTuningPanel::set_parameter_from_slider(const SliderSpec& spec, int slider_value) {
  if (!parameters_client_) {
    return;
  }

  const double value = from_slider_units(spec, slider_value);
  if (!parameters_client_->service_is_ready()) {
    status_label_->setText("Controller parameter service not ready");
    return;
  }

  std::vector<rclcpp::Parameter> parameters;
  parameters.emplace_back(spec.parameter_name, value);

  auto future = parameters_client_->set_parameters(parameters);
  if (executor_->spin_until_future_complete(future, 2s) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    status_label_->setText(
        QString("Timed out while updating %1").arg(QString::fromStdString(spec.parameter_name)));
    return;
  }
  const auto results = future.get();
  if (!results.empty() && results.front().successful) {
    status_label_->setText(
        QString("Updated %1 = %2")
            .arg(QString::fromStdString(spec.parameter_name))
            .arg(QString::fromStdString(format_value(value))));
  } else if (!results.empty()) {
    status_label_->setText(
        QString("Failed to update %1: %2")
            .arg(QString::fromStdString(spec.parameter_name))
            .arg(QString::fromStdString(results.front().reason)));
  } else {
    status_label_->setText("Parameter update returned no result");
  }
}

bool ImpedanceTuningPanel::set_parameters(const std::map<std::string, double>& values,
                                          const std::string& success_label) {
  if (!parameters_client_) {
    return false;
  }

  if (!parameters_client_->service_is_ready()) {
    status_label_->setText("Controller parameter service not ready");
    return false;
  }

  std::vector<rclcpp::Parameter> parameters;
  parameters.reserve(values.size());
  for (const auto& [name, value] : values) {
    parameters.emplace_back(name, value);
  }

  auto future = parameters_client_->set_parameters(parameters);
  if (executor_->spin_until_future_complete(future, 2s) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    status_label_->setText("Timed out while updating parameters");
    return false;
  }

  const auto results = future.get();
  for (std::size_t i = 0; i < results.size(); ++i) {
    if (!results[i].successful) {
      const auto parameter_name = parameters[i].get_name();
      status_label_->setText(QString("Failed to update %1: %2")
                                 .arg(QString::fromStdString(parameter_name))
                                 .arg(QString::fromStdString(results[i].reason)));
      return false;
    }
  }

  status_label_->setText(QString::fromStdString(success_label));
  return true;
}

void ImpedanceTuningPanel::update_slider_display(const std::string& parameter_name, double value) {
  const auto it = sliders_.find(parameter_name);
  if (it == sliders_.end()) {
    return;
  }

  auto& widgets = it->second;
  const int slider_units = to_slider_units(widgets.spec, value);
  QSignalBlocker blocker(widgets.slider);
  widgets.slider->setValue(slider_units);
  widgets.value_label->setText(QString::fromStdString(format_value(value)));
}

int ImpedanceTuningPanel::to_slider_units(const SliderSpec& spec, double value) {
  const double clamped = std::clamp(value, spec.min_value, spec.max_value);
  return static_cast<int>(std::llround((clamped - spec.min_value) / spec.step));
}

double ImpedanceTuningPanel::from_slider_units(const SliderSpec& spec, int slider_value) {
  return spec.min_value + static_cast<double>(slider_value) * spec.step;
}

void ImpedanceTuningPanel::on_refresh_clicked() { request_current_parameters(); }

void ImpedanceTuningPanel::on_raw_config_clicked() {
  const auto values = raw_config_values();

  updating_from_remote_ = true;
  for (const auto& [name, value] : values) {
    update_slider_display(name, value);
  }
  updating_from_remote_ = false;

  set_parameters(values, "Restored raw_config parameters");
}

void ImpedanceTuningPanel::on_set_defaults_clicked() {
  const auto values = current_slider_values();
  const auto paths = default_config_paths();
  if (paths.empty()) {
    status_label_->setText("Could not find default config path");
    return;
  }

  std::vector<std::string> written_paths;
  for (const auto& path : paths) {
    std::string error;
    if (write_defaults_to_file(path, values, error)) {
      written_paths.push_back(path);
    } else {
      status_label_->setText(QString("Failed to save defaults: %1")
                                 .arg(QString::fromStdString(error)));
      return;
    }
  }

  status_label_->setText(
      QString("Saved defaults to %1 config file(s)").arg(static_cast<int>(written_paths.size())));
}

void ImpedanceTuningPanel::on_target_node_changed() {
  if (!node_) {
    return;
  }
  parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
      node_, target_node_edit_->text().toStdString());
  request_current_parameters();
}

std::map<std::string, double> ImpedanceTuningPanel::current_slider_values() const {
  std::map<std::string, double> values;
  for (const auto& [name, widgets] : sliders_) {
    values[name] = from_slider_units(widgets.spec, widgets.slider->value());
  }
  return values;
}

std::map<std::string, double> ImpedanceTuningPanel::raw_config_values() const {
  return {
      {"translational_stiffness", 2000.0},
      {"translational_damping", 89.0},
      {"rotational_stiffness", 300.0},
      {"rotational_damping", 7.0},
      {"nullspace_stiffness", 0.5},
      {"joint1_nullspace_stiffness", 100.0},
      {"translational_clip_x", 0.03},
      {"translational_clip_y", 0.03},
      {"translational_clip_z", 0.03},
      {"translational_clip_neg_x", 0.03},
      {"translational_clip_neg_y", 0.03},
      {"translational_clip_neg_z", 0.03},
      {"rotational_clip_x", 0.05},
      {"rotational_clip_y", 0.05},
      {"rotational_clip_z", 0.05},
      {"rotational_clip_neg_x", 0.05},
      {"rotational_clip_neg_y", 0.05},
      {"rotational_clip_neg_z", 0.05},
      {"translational_ki", 0.0},
      {"rotational_ki", 0.0},
      {"filter_params", 0.02},
  };
}

bool ImpedanceTuningPanel::write_defaults_to_file(
    const std::string& file_path,
    const std::map<std::string, double>& values,
    std::string& error) const {
  std::ifstream input(file_path);
  if (!input.is_open()) {
    error = "cannot open " + file_path;
    return false;
  }

  std::vector<std::string> lines;
  std::string line;
  while (std::getline(input, line)) {
    lines.push_back(line);
  }

  bool in_controller = false;
  bool changed = false;
  for (auto& yaml_line : lines) {
    if (yaml_line == "  cartesian_impedance_controller:") {
      in_controller = true;
      continue;
    }
    if (in_controller && yaml_line.rfind("  ", 0) == 0 &&
        yaml_line.rfind("      ", 0) != 0 && yaml_line != "    ros__parameters:") {
      in_controller = false;
    }
    if (!in_controller || yaml_line.rfind("      ", 0) != 0) {
      continue;
    }

    const auto separator = yaml_line.find(':');
    if (separator == std::string::npos) {
      continue;
    }
    const auto parameter_name = yaml_line.substr(6, separator - 6);
    const auto it = values.find(parameter_name);
    if (it == values.end()) {
      continue;
    }

    yaml_line = "      " + parameter_name + ": " + yaml_value(it->second);
    changed = true;
  }

  if (!changed) {
    error = "no impedance parameters were updated in " + file_path;
    return false;
  }

  std::ofstream output(file_path, std::ios::trunc);
  if (!output.is_open()) {
    error = "cannot write " + file_path;
    return false;
  }

  for (const auto& output_line : lines) {
    output << output_line << '\n';
  }
  return true;
}

std::vector<std::string> ImpedanceTuningPanel::default_config_paths() const {
  std::vector<std::string> paths;
  try {
    const auto share_dir = ament_index_cpp::get_package_share_directory(
        "serl_franka_controllers_ros2");
    const auto install_config =
        std::filesystem::path(share_dir) / "config" / "serl_franka_controllers.yaml";
    if (std::filesystem::exists(install_config)) {
      paths.push_back(install_config.string());
    }

    const auto share_path = std::filesystem::path(share_dir);
    for (auto candidate = share_path; !candidate.empty(); candidate = candidate.parent_path()) {
      if (candidate.filename() != "install") {
        continue;
      }
      const auto source_config = candidate.parent_path() / "src" / "serl_franka_controllers_ros2" /
                                 "config" / "serl_franka_controllers.yaml";
      if (std::filesystem::exists(source_config) &&
          std::find(paths.begin(), paths.end(), source_config.string()) == paths.end()) {
        paths.push_back(source_config.string());
      }
      break;
    }
  } catch (const std::exception& e) {
    (void)e;
  }
  return paths;
}

}  // namespace serl_franka_controllers_ros2

PLUGINLIB_EXPORT_CLASS(serl_franka_controllers_ros2::ImpedanceTuningPanel, rviz_common::Panel)
