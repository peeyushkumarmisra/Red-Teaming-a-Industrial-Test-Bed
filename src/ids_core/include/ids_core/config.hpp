#pragma once

#include <string>

#include "iiwa_control_law/control_law.hpp"
#include "iiwa_control_law/robot_model.hpp"

namespace ids_core {

enum class InputSource { kTrustedReference, kCommandedTorque };
enum class EstimatorMode { kThesis, kExtended };

struct ModelConfig {
  double controller_period = 0.001;
  double reference_hold = 0.05;
  double reference_latency = 0.0;
  std::string tip_joint = "joint_a7";
  iiwa_control_law::Gains gains = iiwa_control_law::Gains::iiwaDefault();
  iiwa_control_law::Payload nominal_payload = defaultPayload();

  static iiwa_control_law::Payload defaultPayload() { return iiwa_control_law::defaultPayload(); }
};

struct EkfConfig {
  double q_position = 1e-11;
  double q_velocity = 1e-7;
  double q_friction = 1e-9;
  double r_position = 1e-10;
  double r_velocity = 1e-6;
  double p0_position = 1e-8;
  double p0_velocity = 1e-6;
  double p0_friction = 1.0;
  double friction_max = 50.0;
  double nis_gate = 36.1;
  int rollback_steps = 50;
  int refine_iterations = 1;
};

struct EwmaConfig {
  double tau_fast = 0.0448;
  double tau_slow = 1.995;
};

struct DetectorConfig {
  double false_alarm_budget_per_hour = 0.125;
  int debounce = 5;
  int clear_count = 5;
  double hysteresis = 0.5;
  bool latch = true;
  bool use_slow_detector = true;
  bool use_command_check = true;
  double limit_override = 0.0;
  double slow_limit_override = 0.0;
  double sigma_floor = 1e-3;
};

struct IdsConfig {
  InputSource input_source = InputSource::kTrustedReference;
  EstimatorMode mode = EstimatorMode::kExtended;
  double nominal_rate_hz = 100.0;
  int max_steps_per_window = 100;
  double context_guard = 0.3;
  ModelConfig model;
  EkfConfig ekf;
  EwmaConfig ewma;
  DetectorConfig detector;
};

}  // namespace ids_core