#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "ids_core/calibration.hpp"
#include "ids_core/ids_pipeline.hpp"
#include "ids_interfaces/msg/ids_status.hpp"
#include "ids_interfaces/msg/joint_measurement.hpp"
#include "ids_interfaces/msg/joint_reference.hpp"
#include "ids_ros/conversions.hpp"
#include "ids_ros/parameters.hpp"

namespace ids_ros {

namespace {

using ids_interfaces::msg::IdsStatus;
using ids_interfaces::msg::JointMeasurement;
using ids_interfaces::msg::JointReference;

rclcpp::ParameterValue toRos(const ParameterValue& v) {
  return std::visit([](const auto& x) { return rclcpp::ParameterValue(x); }, v);
}

ParameterValue fromRos(const rclcpp::Parameter& p) {
  switch (p.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL:
      return p.as_bool();
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return static_cast<std::int64_t>(p.as_int());
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return p.as_double();
    case rclcpp::ParameterType::PARAMETER_STRING:
      return p.as_string();
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
      return p.as_double_array();
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY: {
      const auto a = p.as_integer_array();
      return std::vector<double>(a.begin(), a.end());
    }
    default:
      throw std::invalid_argument("unsupported type for parameter " + p.get_name());
  }
}

std::string readFile(const std::string& path) {
  std::ifstream f(path);
  if (!f) {
    throw std::runtime_error("cannot read model file: " + path);
  }
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

std::string maskString(const ids_core::Mask7& m) {
  std::string s;
  for (int i = 0; i < ids_core::kDof; ++i) {
    s += m(i) ? '1' : '0';
  }
  return s;
}

}  // namespace

class IdsNode : public rclcpp::Node {
 public:
  explicit IdsNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("ids_node", options) {
    rcl_interfaces::msg::ParameterDescriptor flexible;
    flexible.dynamic_typing = true;
    ParameterMap values;
    for (const auto& [name, value] : defaultParameters()) {
      declare_parameter(name, toRos(value), flexible);
      values[name] = fromRos(get_parameter(name));
    }
    config_ = configFromParameters(values);
    run_mode_ = declare_parameter<std::string>("run_mode", "armed");
    calibration_file_ = declare_parameter<std::string>("calibration_file", "");
    calibration_duration_ = declare_parameter<double>("calibration_duration", 120.0);
    arm_after_calibration_ = declare_parameter<bool>("arm_after_calibration", false);
    exit_after_calibration_ = declare_parameter<bool>("exit_after_calibration", false);
    warmup_ = declare_parameter<double>("warmup", 2.0);
    const std::string model_urdf = declare_parameter<std::string>("model_urdf", "");
    if (run_mode_ != "armed" && run_mode_ != "calibrate") {
      throw std::invalid_argument("run_mode must be 'armed' or 'calibrate'");
    }
    if (calibration_file_.empty()) {
      throw std::invalid_argument("calibration_file must be set");
    }

    alarm_pub_ = create_publisher<std_msgs::msg::Bool>(
        "alarm", rclcpp::QoS(1).reliable().transient_local());
    status_pub_ = create_publisher<IdsStatus>("status", rclcpp::QoS(100).reliable());
    reset_srv_ = create_service<std_srvs::srv::Trigger>(
        "reset", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          onReset(*response);
        });
    const auto stream = rclcpp::QoS(100).reliable();
    reference_sub_ = create_subscription<JointReference>(
        "/plc/reference", stream,
        [this](JointReference::ConstSharedPtr msg) { onReference(*msg); });
    measurement_sub_ = create_subscription<JointMeasurement>(
        "/robot/measurement", stream,
        [this](JointMeasurement::ConstSharedPtr msg) { onMeasurement(*msg); });
    publishAlarm(false);
    if (!model_urdf.empty()) {
      start(readFile(model_urdf));
    } else {
      description_sub_ = create_subscription<std_msgs::msg::String>(
          "/robot_description", rclcpp::QoS(1).reliable().transient_local(),
          [this](std_msgs::msg::String::ConstSharedPtr msg) {
            if (!pipeline_) {
              start(msg->data);
            }
          });
      RCLCPP_INFO(get_logger(), "waiting for /robot_description");
    }
  }

 private:
  void start(const std::string& urdf) {
    urdf_ = urdf;
    fault_ = false;
    publishAlarm(false);
    have_reference_ = false;
    last_reference_seq_ = 0;
    first_reference_stamp_ = -1.0;
    last_stamp_ = -1.0;
    calibration_first_stamp_ = -1.0;
    try {
      pipeline_ = std::make_unique<ids_core::IdsPipeline>(config_, urdf);
    } catch (const std::exception& e) {
      pipeline_.reset();
      fault_ = true;
      RCLCPP_ERROR(get_logger(), "cannot build the IDS model: %s", e.what());
      return;
    }
    RCLCPP_INFO(get_logger(), "model fingerprint %s", pipeline_->fingerprint().c_str());
    if (run_mode_ == "calibrate") {
      pipeline_->startCalibration();
      RCLCPP_INFO(get_logger(), "calibrating for %.1f s of sim time after a %.1f s warm-up",
                  calibration_duration_, warmup_);
      return;
    }
    try {
      pipeline_->arm(ids_core::loadCalibration(calibration_file_));
      const auto& cal = pipeline_->calibration();
      RCLCPP_INFO(get_logger(), "armed from %s: limit %.3f, slow limit %.3f",
                  calibration_file_.c_str(), cal.limit, cal.slow_limit);
    } catch (const std::exception& e) {
      fault_ = true;
      RCLCPP_ERROR(get_logger(), "refusing to arm: %s", e.what());
    }
  }

  void onReference(const JointReference& msg) {
    if (!pipeline_) {
      return;
    }
    if (have_reference_ && msg.seq <= last_reference_seq_) {
      return;
    }
    const double stamp = rclcpp::Time(msg.header.stamp).seconds();
    pipeline_->addReference(toReferenceSample(msg, stamp));
    if (!have_reference_) {
      first_reference_stamp_ = stamp;
      have_reference_ = true;
    }
    last_reference_seq_ = msg.seq;
  }

  void onMeasurement(const JointMeasurement& msg) {
    if (!pipeline_ || !have_reference_) {
      return;
    }
    const double stamp = rclcpp::Time(msg.header.stamp).seconds();
    if (last_stamp_ >= 0.0 && stamp < last_stamp_ - 1.0) {
      RCLCPP_WARN(get_logger(), "sim time jumped back by %.2f s; restarting the IDS",
                  last_stamp_ - stamp);
      start(urdf_);
      return;
    }
    if (stamp < first_reference_stamp_ + warmup_) {
      return;
    }
    last_stamp_ = stamp;
    const auto t0 = std::chrono::steady_clock::now();
    const ids_core::IdsOutput& out = pipeline_->process(toMeasurement(msg, stamp));
    const double us =
        std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
    IdsStatus status;
    status.header.stamp = msg.header.stamp;
    fillStatus(out, us, status);
    if (fault_) {
      status.mode = IdsStatus::MODE_FAULT;
    }
    status_pub_->publish(status);
    if (!out.valid) {
      return;
    }
    if (out.mode == ids_core::Mode::kCalibrating) {
      handleCalibration(stamp);
      return;
    }
    const bool alarm = out.alarm || out.latched;
    if (alarm && !alarm_state_) {
      RCLCPP_WARN(get_logger(), "ALARM at t=%.3f s: primary joint a%d, physics %s, command %s",
                  stamp, out.primary_joint + 1, maskString(out.physics_alarm).c_str(),
                  maskString(out.command_alarm).c_str());
    }
    if (alarm != alarm_state_) {
      publishAlarm(alarm);
    }
  }

  void handleCalibration(double stamp) {
    if (calibration_first_stamp_ < 0.0) {
      calibration_first_stamp_ = stamp;
    }
    if (stamp - calibration_first_stamp_ < calibration_duration_) {
      return;
    }
    try {
      const ids_core::Calibration cal = pipeline_->finishCalibration();
      const auto parent = std::filesystem::path(calibration_file_).parent_path();
      if (!parent.empty()) {
        std::filesystem::create_directories(parent);
      }
      ids_core::saveCalibration(cal, calibration_file_);
      RCLCPP_INFO(get_logger(),
                  "calibration saved to %s: %ld samples over %.1f s, limit %.3f, slow limit %.3f",
                  calibration_file_.c_str(), cal.samples, cal.duration_s, cal.limit,
                  cal.slow_limit);
      if (arm_after_calibration_) {
        pipeline_->arm(cal);
        RCLCPP_INFO(get_logger(), "armed with the new calibration");
      }
    } catch (const std::exception& e) {
      fault_ = true;
      RCLCPP_ERROR(get_logger(), "calibration failed: %s", e.what());
    }
    if (exit_after_calibration_) {
      rclcpp::shutdown();
    }
  }

  void onReset(std_srvs::srv::Trigger::Response& response) {
    if (!pipeline_ || pipeline_->mode() != ids_core::Mode::kArmed) {
      response.success = false;
      response.message = "IDS is not armed";
      return;
    }
    pipeline_->resetAlarm();
    publishAlarm(false);
    response.success = true;
    response.message = "alarm reset";
    RCLCPP_INFO(get_logger(), "alarm reset");
  }

  void publishAlarm(bool alarm) {
    alarm_state_ = alarm;
    std_msgs::msg::Bool msg;
    msg.data = alarm;
    alarm_pub_->publish(msg);
  }

  ids_core::IdsConfig config_;
  std::unique_ptr<ids_core::IdsPipeline> pipeline_;
  std::string urdf_;
  std::string run_mode_;
  std::string calibration_file_;
  double calibration_duration_ = 120.0;
  double warmup_ = 2.0;
  bool arm_after_calibration_ = false;
  bool exit_after_calibration_ = false;
  bool fault_ = false;
  bool alarm_state_ = false;
  bool have_reference_ = false;
  std::uint64_t last_reference_seq_ = 0;
  double first_reference_stamp_ = -1.0;
  double last_stamp_ = -1.0;
  double calibration_first_stamp_ = -1.0;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr alarm_pub_;
  rclcpp::Publisher<IdsStatus>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::Subscription<JointReference>::SharedPtr reference_sub_;
  rclcpp::Subscription<JointMeasurement>::SharedPtr measurement_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_sub_;
};

}  // namespace ids_ros

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    rclcpp::spin(std::make_shared<ids_ros::IdsNode>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("ids_node"), "%s", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}