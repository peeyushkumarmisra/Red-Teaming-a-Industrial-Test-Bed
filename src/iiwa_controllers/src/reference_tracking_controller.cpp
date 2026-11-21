// =============================================================================
//  reference_tracking_controller.cpp - ros2_control plugin implementation
// -----------------------------------------------------------------------------
//  on_init: declare parameters (defaults equal the IDS's). on_configure: build the
//  gravity model from /robot_description and check joint names. on_activate: claim the
//  interfaces. update (1 kHz): read joints -> encoder model -> tracking core -> write
//  torques -> every 10th tick publish a measurement for the IDS.
//  Hardware interface access uses the current Jazzy API (get_optional / checked
//  set_value, B12): a failed read skips the cycle and is reported, never ignored.
// =============================================================================

#include "iiwa_controllers/reference_tracking_controller.hpp"

#include <cmath>
#include <cstdint>
#include <exception>
#include <string>
#include <vector>

#include <controller_interface/helpers.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace iiwa_controllers {

namespace {

std::vector<double> listOf(const Vec7& v) { return {v.data(), v.data() + iiwa_control_law::kDof}; }

bool toVec7(const std::vector<double>& l, Vec7& out) {
  if (l.size() != static_cast<std::size_t>(iiwa_control_law::kDof)) {
    return false;
  }
  for (int i = 0; i < iiwa_control_law::kDof; ++i) {
    out(i) = l[static_cast<std::size_t>(i)];
  }
  return true;
}

}  // namespace

// Parameters and their defaults (see docs/CONFIGURATION.md).
controller_interface::CallbackReturn ReferenceTrackingController::on_init() {
  try {
    const TrackingConfig d;
    const EncoderConfig e;
    auto_declare<std::vector<std::string>>(
        "joints", {"joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6",
                   "joint_a7"});
    auto_declare<std::string>("tip_joint", "joint_a7");
    auto_declare<std::vector<double>>("kp", listOf(d.gains.kp));
    auto_declare<std::vector<double>>("kd", listOf(d.gains.kd));
    auto_declare<std::vector<double>>("effort_limit", listOf(d.gains.effort_limit));
    auto_declare<std::vector<double>>(
        "payload_com", {d.nominal_payload.com.x(), d.nominal_payload.com.y(),
                        d.nominal_payload.com.z()});
    auto_declare<std::vector<double>>(
        "payload_inertia",
        {d.nominal_payload.inertia(0, 0), d.nominal_payload.inertia(1, 1),
         d.nominal_payload.inertia(2, 2)});
    auto_declare<double>("reference_hold", d.reference_hold);
    auto_declare<double>("reference_timeout", d.reference_timeout);
    auto_declare<double>("resume_time", d.resume_time);
    auto_declare<bool>("halt_on_alarm", d.halt_on_alarm);
    // true in simulation: count ticks instead of reading the late /clock (B19).
    auto_declare<bool>("tick_clock", false);
    auto_declare<int>("encoder_bits", e.position_bits);
    auto_declare<double>("velocity_noise", e.velocity_noise);
    // Set per run by the experiment launch, so seeds 1, 2, 3 give different sensor noise.
    auto_declare<int>("noise_seed", static_cast<int>(e.seed));
    // One measurement every 10 ticks = one IDS window every 10 ms.
    auto_declare<int>("measurement_decimation", 10);
    auto_declare<std::string>("reference_topic", "/robot/reference");
    auto_declare<std::string>("measurement_topic", "/robot/measurement");
    auto_declare<std::string>("alarm_topic", "/ids/alarm");
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(get_node()->get_logger(), "parameter declaration failed: %s", ex.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
// Effort (torque) command for each joint.
ReferenceTrackingController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration c;
  c.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& j : joints_) {
    c.names.push_back(j + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return c;
}

controller_interface::InterfaceConfiguration
// Position and velocity of each joint.
ReferenceTrackingController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration c;
  c.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& j : joints_) {
    c.names.push_back(j + "/" + hardware_interface::HW_IF_POSITION);
  }
  for (const auto& j : joints_) {
    c.names.push_back(j + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return c;
}

// Check parameters, build the gravity model and the ROS connections.
controller_interface::CallbackReturn ReferenceTrackingController::on_configure(
    const rclcpp_lifecycle::State&) {
  auto node = get_node();
  const auto log = node->get_logger();
  joints_ = node->get_parameter("joints").as_string_array();
  if (joints_.size() != static_cast<std::size_t>(iiwa_control_law::kDof)) {
    RCLCPP_ERROR(log, "expected 7 joints, got %zu", joints_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  TrackingConfig c;
  if (!toVec7(node->get_parameter("kp").as_double_array(), c.gains.kp) ||
      !toVec7(node->get_parameter("kd").as_double_array(), c.gains.kd) ||
      !toVec7(node->get_parameter("effort_limit").as_double_array(), c.gains.effort_limit)) {
    RCLCPP_ERROR(log, "kp, kd and effort_limit must have 7 entries each");
    return controller_interface::CallbackReturn::ERROR;
  }
  const auto com = node->get_parameter("payload_com").as_double_array();
  const auto inertia = node->get_parameter("payload_inertia").as_double_array();
  if (com.size() != 3 || inertia.size() != 3) {
    RCLCPP_ERROR(log, "payload_com and payload_inertia must have 3 entries each");
    return controller_interface::CallbackReturn::ERROR;
  }
  c.nominal_payload.com = Eigen::Vector3d(com[0], com[1], com[2]);
  c.nominal_payload.inertia = Eigen::Vector3d(inertia[0], inertia[1], inertia[2]).asDiagonal();
  c.reference_hold = node->get_parameter("reference_hold").as_double();
  c.reference_timeout = node->get_parameter("reference_timeout").as_double();
  c.resume_time = node->get_parameter("resume_time").as_double();
  c.halt_on_alarm = node->get_parameter("halt_on_alarm").as_bool();
  c.nominal_period = 1.0 / static_cast<double>(get_update_rate() > 0 ? get_update_rate() : 1000);
  tracking_config_ = c;
  use_tick_clock_ = node->get_parameter("tick_clock").as_bool();
  tick_clock_ = TickClock(static_cast<std::int64_t>(std::llround(c.nominal_period * 1e9)));
  encoder_config_.position_bits = static_cast<int>(node->get_parameter("encoder_bits").as_int());
  encoder_config_.velocity_noise = node->get_parameter("velocity_noise").as_double();
  encoder_config_.seed = static_cast<std::uint64_t>(node->get_parameter("noise_seed").as_int());
  decimation_ = static_cast<int>(node->get_parameter("measurement_decimation").as_int());
  if (decimation_ < 1) {
    RCLCPP_ERROR(log, "measurement_decimation must be at least 1");
    return controller_interface::CallbackReturn::ERROR;
  }

  const std::string& description = get_robot_description();
  if (description.empty()) {
    RCLCPP_ERROR(log, "no robot description available to build the gravity model");
    return controller_interface::CallbackReturn::ERROR;
  }
  try {
    model_.emplace(iiwa_control_law::RobotModel::fromUrdfString(
        description, node->get_parameter("tip_joint").as_string()));
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(log, "cannot build the gravity model: %s", ex.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  for (int i = 0; i < iiwa_control_law::kDof; ++i) {
    if (model_->jointNames()[static_cast<std::size_t>(i)] != joints_[static_cast<std::size_t>(i)]) {
      RCLCPP_ERROR(log, "joint %d is %s in the model but %s in the controller", i,
                   model_->jointNames()[static_cast<std::size_t>(i)].c_str(),
                   joints_[static_cast<std::size_t>(i)].c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  reference_buffer_.writeFromNonRT(nullptr);
  const auto stream = rclcpp::QoS(100).reliable();
  // Reference from the network (may have been changed by the MitM).
  reference_sub_ = node->create_subscription<Reference>(
      node->get_parameter("reference_topic").as_string(), stream,
      [this](const std::shared_ptr<Reference> msg) { reference_buffer_.writeFromNonRT(msg); });
  // IDS alarm (latched, transient-local) for the optional halt.
  alarm_sub_ = node->create_subscription<std_msgs::msg::Bool>(
      node->get_parameter("alarm_topic").as_string(), rclcpp::QoS(1).reliable().transient_local(),
      [this](const std::shared_ptr<std_msgs::msg::Bool> msg) {
        alarm_.store(msg->data, std::memory_order_relaxed);
      });
  measurement_pub_ = node->create_publisher<Measurement>(
      node->get_parameter("measurement_topic").as_string(), stream);
  measurement_rt_ =
      std::make_unique<realtime_tools::RealtimePublisher<Measurement>>(measurement_pub_);
  RCLCPP_INFO(log,
              "configured: halt_on_alarm=%s, encoder %d bit, velocity noise %.2e rad/s, "
              "time from %s",
              c.halt_on_alarm ? "true" : "false", encoder_config_.position_bits,
              encoder_config_.velocity_noise,
              use_tick_clock_ ? "update count" : "controller manager");
  return controller_interface::CallbackReturn::SUCCESS;
}

// Claim the interfaces and start a fresh timeline.
controller_interface::CallbackReturn ReferenceTrackingController::on_activate(
    const rclcpp_lifecycle::State&) {
  position_states_.clear();
  velocity_states_.clear();
  effort_commands_.clear();
  if (!controller_interface::get_ordered_interfaces(state_interfaces_, joints_,
                                                    hardware_interface::HW_IF_POSITION,
                                                    position_states_) ||
      !controller_interface::get_ordered_interfaces(state_interfaces_, joints_,
                                                    hardware_interface::HW_IF_VELOCITY,
                                                    velocity_states_) ||
      !controller_interface::get_ordered_interfaces(command_interfaces_, joints_,
                                                    hardware_interface::HW_IF_EFFORT,
                                                    effort_commands_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "missing joint interfaces");
    return controller_interface::CallbackReturn::ERROR;
  }
  core_ = std::make_unique<TrackingCore>(*model_, tracking_config_);
  encoder_ = std::make_unique<EncoderModel>(encoder_config_);
  reference_buffer_.writeFromNonRT(nullptr);
  have_seq_ = false;
  decimator_ = MeasurementDecimator(decimation_);
  tick_clock_.reset();
  reported_overruns_ = 0;
  io_failures_ = 0;
  reported_io_failures_ = 0;
  return controller_interface::CallbackReturn::SUCCESS;
}

// Leave zero torque behind when the controller stops.
controller_interface::CallbackReturn ReferenceTrackingController::on_deactivate(
    const rclcpp_lifecycle::State&) {
  for (auto& c : effort_commands_) {
    if (!c.get().set_value(0.0)) {
      RCLCPP_WARN(get_node()->get_logger(), "could not zero an effort command on deactivation");
    }
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

// One control tick (1 ms).
controller_interface::return_type ReferenceTrackingController::update(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
  rclcpp::Time now = time;
  rclcpp::Duration elapsed = period;
  // Simulation: time = first update + n * 1 ms, exactly as Gazebo stepped (B19).
  if (use_tick_clock_) {
    now = rclcpp::Time(tick_clock_.next(time.nanoseconds()), time.get_clock_type());
    elapsed = rclcpp::Duration::from_nanoseconds(tick_clock_.period_ns());
  }
  Vec7 q = Vec7::Zero();
  Vec7 v = Vec7::Zero();
  bool read_ok = true;
  for (int i = 0; i < iiwa_control_law::kDof && read_ok; ++i) {
    const auto k = static_cast<std::size_t>(i);
    const std::optional<double> position = position_states_[k].get().get_optional();
    const std::optional<double> velocity = velocity_states_[k].get().get_optional();
    read_ok = position.has_value() && velocity.has_value();
    if (read_ok) {
      q(i) = *position;
      v(i) = *velocity;
    }
  }
  if (read_ok) {
    step(now, elapsed, q, v);
  } else {
    ++io_failures_;
  }
  // Report missed deadlines and I/O failures, throttled to one line per 5 s.
  if (core_->overruns() != reported_overruns_) {
    reported_overruns_ = core_->overruns();
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
                         "%ld control periods over the deadline so far", reported_overruns_);
  }
  if (io_failures_ != reported_io_failures_) {
    reported_io_failures_ = io_failures_;
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
                         "%ld cycles with a failed state read or command write so far",
                         reported_io_failures_);
  }
  return controller_interface::return_type::OK;
}

// Sense, decide, actuate, and publish the measurement window.
void ReferenceTrackingController::step(const rclcpp::Time& time, const rclcpp::Duration& period,
                                       const Vec7& q, const Vec7& v) {
  Vec7 qm;
  Vec7 vm;
  // The controller (and the IDS) only ever see the encoder's view of the arm.
  encoder_->sense(q, v, qm, vm);
  const std::shared_ptr<Reference> ref = *reference_buffer_.readFromRT();
  // A new reference sample arrived: hand it to the tracking core.
  if (ref && (!have_seq_ || ref->seq != last_seq_)) {
    iiwa_control_law::ReferenceSample s;
    s.stamp = rclcpp::Time(ref->header.stamp).seconds();
    s.seq = ref->seq;
    for (int i = 0; i < iiwa_control_law::kDof; ++i) {
      const auto k = static_cast<std::size_t>(i);
      s.position(i) = ref->position[k];
      s.velocity(i) = ref->velocity[k];
      s.acceleration(i) = ref->acceleration[k];
    }
    s.payload_context = ref->payload_context;
    s.payload_mass = ref->payload_mass;
    core_->setReference(s);
    last_seq_ = ref->seq;
    have_seq_ = true;
  }
  const bool alarm = alarm_.load(std::memory_order_relaxed);
  const TrackingOutput& out = core_->update(time.seconds(), period.seconds(), qm, vm, alarm);
  bool written = true;
  for (int i = 0; i < iiwa_control_law::kDof; ++i) {
    const auto k = static_cast<std::size_t>(i);
    written = effort_commands_[k].get().set_value(out.torque(i)) && written;
  }
  if (!written) {
    ++io_failures_;
  }
  bool window_halted = out.halted;
  // Every 10th tick close the IDS window; a busy publisher defers the halt flag.
  const bool publish = decimator_.tick(out.halted, window_halted);
  if (publish && !measurement_rt_->trylock()) {
    decimator_.defer(window_halted);
  } else if (publish) {
    Measurement& m = measurement_rt_->msg_;
    // Stamp = this tick's time: the IDS replays the window up to exactly here.
    m.header.stamp = time;
    for (int i = 0; i < iiwa_control_law::kDof; ++i) {
      const auto k = static_cast<std::size_t>(i);
      m.position[k] = qm(i);
      m.velocity[k] = vm(i);
      m.effort_applied[k] = out.torque(i);
    }
    m.reference_seq = out.reference_seq;
    m.halted = window_halted;
    measurement_rt_->unlockAndPublish();
  }
}

}  // namespace iiwa_controllers

PLUGINLIB_EXPORT_CLASS(iiwa_controllers::ReferenceTrackingController,
                       controller_interface::ControllerInterface)
