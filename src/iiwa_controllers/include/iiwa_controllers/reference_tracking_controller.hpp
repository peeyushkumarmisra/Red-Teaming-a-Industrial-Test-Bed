// =============================================================================
//  reference_tracking_controller.hpp - the ros2_control plugin
// -----------------------------------------------------------------------------
//  Claims the effort command and the position/velocity states of joint_a1..a7.
//  Subscribes to /robot/reference (after the MitM) and /ids/alarm; publishes
//  /robot/measurement every 10 ms. Real-time safe: the update loop never blocks
//  (lock-free buffer in, real-time publisher out).
// =============================================================================

#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <std_msgs/msg/bool.hpp>

#include "ids_interfaces/msg/joint_measurement.hpp"
#include "ids_interfaces/msg/joint_reference.hpp"
#include "iiwa_controllers/encoder_model.hpp"
#include "iiwa_controllers/tracking_core.hpp"

namespace iiwa_controllers {

// Runs inside controller_manager at 1 kHz.
class ReferenceTrackingController : public controller_interface::ControllerInterface {
 public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous) override;
  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous) override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

 private:
  void step(const rclcpp::Time& time, const rclcpp::Duration& period, const Vec7& q,
            const Vec7& v);

  using Reference = ids_interfaces::msg::JointReference;
  using Measurement = ids_interfaces::msg::JointMeasurement;

  std::vector<std::string> joints_;
  TrackingConfig tracking_config_;
  EncoderConfig encoder_config_;
  int decimation_ = 10;
  std::optional<iiwa_control_law::RobotModel> model_;
  std::unique_ptr<TrackingCore> core_;
  std::unique_ptr<EncoderModel> encoder_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> position_states_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> velocity_states_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> effort_commands_;
  // Latest reference, handed from the ROS thread to the real-time loop without locking.
  realtime_tools::RealtimeBuffer<std::shared_ptr<Reference>> reference_buffer_;
  std::atomic<bool> alarm_{false};
  rclcpp::Subscription<Reference>::SharedPtr reference_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alarm_sub_;
  rclcpp::Publisher<Measurement>::SharedPtr measurement_pub_;
  // Publishes from the real-time loop without blocking it.
  std::unique_ptr<realtime_tools::RealtimePublisher<Measurement>> measurement_rt_;
  MeasurementDecimator decimator_{10};
  // Exact 1 ms timeline in simulation (parameter tick_clock).
  TickClock tick_clock_;
  bool use_tick_clock_ = false;
  std::uint64_t last_seq_ = 0;
  bool have_seq_ = false;
  long reported_overruns_ = 0;
  long io_failures_ = 0;
  long reported_io_failures_ = 0;
};

}  // namespace iiwa_controllers
