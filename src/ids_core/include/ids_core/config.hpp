// =============================================================================
//  config.hpp - every setting of the IDS in one place
// -----------------------------------------------------------------------------
//  Defaults here are the values used for the v1.2 results. Each field is also a ROS
//  parameter with the same dotted name (see ids_ros/parameters.cpp). All fields that
//  change detection are part of the calibration fingerprint, so a calibration can never
//  be reused with different settings.
// =============================================================================

#pragma once

#include <string>

#include "iiwa_control_law/control_law.hpp"
#include "iiwa_control_law/robot_model.hpp"

namespace ids_core {

// What drives the model: the trusted PLC reference (v2) or the commanded torque
// (the dissertation's design, which an attacker on the setpoint path also controls).
enum class InputSource { kTrustedReference, kCommandedTorque };
// Thesis: no friction states (dissertation). Extended: friction learned per joint (v2).
enum class EstimatorMode { kThesis, kExtended };

// How the IDS replays the controller. Must match the controller's settings exactly.
struct ModelConfig {
  // The controller runs at 1 kHz: 10 ticks per 10 ms measurement window.
  double controller_period = 0.001;
  // How long a reference sample is extrapolated before holding (same as the controller).
  double reference_hold = 0.05;
  double reference_latency = 0.0;
  std::string tip_joint = "joint_a7";
  iiwa_control_law::Gains gains = iiwa_control_law::Gains::iiwaDefault();
  iiwa_control_law::Payload nominal_payload = defaultPayload();

  static iiwa_control_law::Payload defaultPayload() { return iiwa_control_law::defaultPayload(); }
};

// Kalman filter noise settings. q_* = how much the model may be wrong per tick,
// r_* = encoder noise, p0_* = initial uncertainty.
struct EkfConfig {
  double q_position = 1e-11;
  double q_velocity = 1e-7;
  // Small: friction should change slowly (wear), not jump. See open item B24 on its size.
  double q_friction = 1e-9;
  double r_position = 1e-10;
  double r_velocity = 1e-6;
  double p0_position = 1e-8;
  double p0_velocity = 1e-6;
  double p0_friction = 1.0;
  double friction_max = 50.0;
  // Friction is only learned in windows the model explains well (chi-square, 14 dof,
  // about 0.1 % tail), so an attack is not learned as friction.
  double nis_gate = 36.1;
  // At an alarm onset, undo the last 50 windows (0.5 s) of learning, which may already
  // contain the start of the attack.
  int rollback_steps = 50;
  // One Gauss-Newton refinement of the disturbance estimate (removes cross-talk, B3).
  int refine_iterations = 1;
};

// Time constants of the two averages: lambda = 0.2 (fast) and 0.005 (slow) at 100 Hz,
// the dissertation's values.
struct EwmaConfig {
  double tau_fast = 0.0448;
  double tau_slow = 1.995;
};

// How readings become alarms.
struct DetectorConfig {
  // Target: at most one false alarm per 8 hours, shared by all channels and joints.
  double false_alarm_budget_per_hour = 0.125;
  // 5 windows (50 ms) above the limit before an alarm: one noisy window is not enough.
  int debounce = 5;
  int clear_count = 5;
  // An alarm clears after clear_count windows below half the limit.
  double hysteresis = 0.5;
  bool latch = true;
  bool use_slow_detector = true;
  bool use_command_check = true;
  double limit_override = 0.0;
  double slow_limit_override = 0.0;
  double sigma_floor = 1e-3;
};

// Top-level configuration.
struct IdsConfig {
  InputSource input_source = InputSource::kTrustedReference;
  EstimatorMode mode = EstimatorMode::kExtended;
  double nominal_rate_hz = 100.0;
  int max_steps_per_window = 100;
  // For 0.3 s after the trusted payload context changes (grab or release), re-sync
  // instead of detecting: the physical handover is not modelled (B9).
  double context_guard = 0.3;
  ModelConfig model;
  EkfConfig ekf;
  EwmaConfig ewma;
  DetectorConfig detector;
};

}  // namespace ids_core
