#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <QWidget>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rviz_common/panel.hpp>

class QLabel;
class QLineEdit;
class QPushButton;
class QSlider;
class QTimer;
class QVBoxLayout;

namespace serl_franka_controllers_ros2 {

class ImpedanceTuningPanel : public rviz_common::Panel {
 public:
  explicit ImpedanceTuningPanel(QWidget* parent = nullptr);
  ~ImpedanceTuningPanel() override;

  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config& config) override;

 private:
  void on_refresh_clicked();
  void on_raw_config_clicked();
  void on_set_defaults_clicked();
  void on_target_node_changed();
  struct SliderSpec {
    std::string parameter_name;
    double min_value;
    double max_value;
    double step;
    double default_value;
    std::string label;
  };

  struct SliderWidgets {
    SliderSpec spec;
    QLabel* value_label{nullptr};
    QSlider* slider{nullptr};
  };

  void setup_ros();
  void build_ui();
  void add_slider(QVBoxLayout* layout, const SliderSpec& spec);
  void request_current_parameters();
  void set_parameter_from_slider(const SliderSpec& spec, int slider_value);
  bool set_parameters(const std::map<std::string, double>& values, const std::string& success_label);
  void update_slider_display(const std::string& parameter_name, double value);
  std::map<std::string, double> current_slider_values() const;
  std::map<std::string, double> raw_config_values() const;
  bool write_defaults_to_file(const std::string& file_path,
                              const std::map<std::string, double>& values,
                              std::string& error) const;
  std::vector<std::string> default_config_paths() const;
  static int to_slider_units(const SliderSpec& spec, double value);
  static double from_slider_units(const SliderSpec& spec, int slider_value);

  QLineEdit* target_node_edit_{nullptr};
  QPushButton* refresh_button_{nullptr};
  QPushButton* raw_config_button_{nullptr};
  QPushButton* set_defaults_button_{nullptr};
  QLabel* status_label_{nullptr};
  QTimer* spin_timer_{nullptr};

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::shared_ptr<rclcpp::AsyncParametersClient> parameters_client_;

  std::map<std::string, SliderWidgets> sliders_;
  bool updating_from_remote_{false};
};

}  // namespace serl_franka_controllers_ros2
