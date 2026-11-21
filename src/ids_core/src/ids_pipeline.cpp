#include "ids_core/ids_pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ids_core {

IdsPipeline::IdsPipeline(const IdsConfig& config, const std::string& urdf)
    : config_(config),
      fingerprint_(computeFingerprint(urdf, config)),
      robot_(iiwa_control_law::RobotModel::fromUrdfString(urdf, config.model.tip_joint)),
      model_(robot_, config.model, config.input_source),
      ekf_(model_, config.ekf, config.mode) {
  const double dt = 1.0 / config_.nominal_rate_hz;
  physics_ewma_.configure(dt, config_.ewma.tau_fast, config_.ewma.tau_slow);
  command_ewma_.configure(dt, config_.ewma.tau_fast, config_.ewma.tau_slow);
  use_slow_ = config_.mode == EstimatorMode::kExtended && config_.detector.use_slow_detector;
  use_command_ = config_.mode == EstimatorMode::kExtended && config_.detector.use_command_check;
  detector_warmup_ = static_cast<int>(std::ceil(2.0 * config_.ewma.tau_slow / dt));
}

void IdsPipeline::startCalibration() {
  recorder_.clear();
  calibration_start_ = -1.0;
  mode_ = Mode::kCalibrating;
}

Calibration IdsPipeline::finishCalibration() {
  if (mode_ != Mode::kCalibrating || recorder_.samples() < 100) {
    throw std::runtime_error("calibration needs at least 100 samples");
  }
  const Calibration cal = recorder_.finalize(config_, fingerprint_, ekf_.state().b,
                                             last_stamp_ - calibration_start_);
  mode_ = Mode::kWaiting;
  return cal;
}

void IdsPipeline::arm(const Calibration& calibration) {
  if (calibration.fingerprint != fingerprint_) {
    mode_ = Mode::kFault;
    throw std::runtime_error("calibration fingerprint does not match this model and configuration");
  }
  calibration_ = calibration;
  detector_.configure(config_.detector, calibration.limit, calibration.slow_limit);
  physics_ewma_.reset();
  command_ewma_.reset();
  detector_windows_ = 0;
  mode_ = Mode::kArmed;
}

void IdsPipeline::resetAlarm() {
  detector_.reset();
  primary_joint_ = -1;
  physics_ewma_.reset();
  command_ewma_.reset();
  detector_windows_ = 0;
}

void IdsPipeline::addReference(const iiwa_control_law::ReferenceSample& sample) {
  track_.push(sample);
}

void IdsPipeline::applyContext(double t) {
  const iiwa_control_law::ReferenceSample* s = track_.at(t, config_.model.reference_latency);
  if (s == nullptr) {
    return;
  }
  const int context = s->payload_context;
  if (context == context_ && s->payload_mass == context_mass_) {
    return;
  }
  iiwa_control_law::Payload payload = config_.model.nominal_payload;
  payload.mass = context == 0 ? 0.0 : s->payload_mass;
  robot_.setPayload(payload);
  if (context_ >= 0) {
    context_change_ = s->stamp;
  }
  context_ = context;
  context_mass_ = s->payload_mass;
}

const IdsOutput& IdsPipeline::process(const Measurement& m) {
  out_.mode = mode_;
  out_.valid = false;
  if (mode_ == Mode::kWaiting || mode_ == Mode::kFault || track_.empty()) {
    return out_;
  }
  applyContext(m.stamp);
  if (!initialized_) {
    ekf_.initialize(m.q, m.v,
                    mode_ == Mode::kArmed ? calibration_.friction_initial : Vec7::Zero());
    last_stamp_ = m.stamp;
    last_tau_ = m.tau;
    initialized_ = true;
    return out_;
  }
  const int steps = static_cast<int>(
      std::lround((m.stamp - last_stamp_) / config_.model.controller_period));
  if (steps < 1) {
    return out_;
  }
  const bool guarded = m.stamp - context_change_ < config_.context_guard;
  if (m.halted || guarded || steps > config_.max_steps_per_window) {
    ekf_.resync(m.q, m.v);
    last_stamp_ = m.stamp;
    last_tau_ = m.tau;
    return out_;
  }
  const bool alarm_active = detector_.output().alarm;
  ekf_.predict(last_stamp_, steps, track_, last_tau_, alarm_active);
  const EkfOutput& e = ekf_.update(m.q, m.v, !alarm_active);
  last_stamp_ = m.stamp;
  last_tau_ = m.tau;

  Vec7 command_error = Vec7::Zero();
  bool command_ok = false;
  if (!m.halted) {
    const iiwa_control_law::ReferenceSample* rs = track_.bySeq(m.reference_seq);
    if (rs != nullptr) {
      command_error = m.tau - model_.nominalTorque(m.q, m.v, *rs, m.stamp);
      command_ok = true;
    }
  }
  out_.disturbance = e.disturbance;
  out_.command_error = command_error;
  out_.friction = ekf_.state().b;
  out_.nis = e.nis;

  if (mode_ == Mode::kCalibrating) {
    if (calibration_start_ < 0.0) {
      calibration_start_ = m.stamp;
    }
    recorder_.add(e.disturbance.cwiseQuotient(e.disturbance_std),
                  command_ok ? &command_error : nullptr);
    out_.valid = true;
    return out_;
  }

  const Vec7 u = (e.disturbance.cwiseQuotient(e.disturbance_std) - calibration_.physics_mean)
                     .cwiseQuotient(calibration_.physics_std);
  const bool freeze = alarm_active && config_.mode == EstimatorMode::kExtended;
  physics_ewma_.update(u, freeze);
  Vec7 command_stat = Vec7::Zero();
  const bool command_used = use_command_ && command_ok;
  if (command_used) {
    command_ewma_.update(
        (command_error - calibration_.command_mean).cwiseQuotient(calibration_.command_std),
        freeze);
    command_stat = command_ewma_.statistic().cwiseQuotient(calibration_.command_scale);
  }
  const Vec7 physics_stat = physics_ewma_.statistic().cwiseQuotient(calibration_.physics_scale);
  const Vec7 slow_stat = physics_ewma_.slowStatistic().cwiseQuotient(calibration_.slow_scale);
  const bool warming_up = detector_windows_ < detector_warmup_;
  if (warming_up) {
    ++detector_windows_;
  }
  const DetectorOutput& d =
      warming_up ? detector_.update(Vec7::Zero(), Vec7::Zero(), Vec7::Zero(), use_slow_, false)
                 : detector_.update(physics_stat, slow_stat, command_stat, use_slow_,
                                    command_used);
  if (d.onset) {
    if (config_.mode == EstimatorMode::kExtended) {
      ekf_.rollbackFriction(config_.ekf.rollback_steps);
      physics_ewma_.rollbackBaseline(config_.ekf.rollback_steps);
      command_ewma_.rollbackBaseline(config_.ekf.rollback_steps);
    }
    double best = -1.0;
    for (int i = 0; i < kDof; ++i) {
      const double p = std::max(physics_stat(i) / calibration_.limit,
                                slow_stat(i) / calibration_.slow_limit);
      const double score = (d.physics(i) || d.physics_slow(i)) ? p
                           : d.command(i) ? command_stat(i) / calibration_.limit
                                          : -1.0;
      if (score > best) {
        best = score;
        primary_joint_ = i;
      }
    }
  }
  out_.valid = true;
  out_.alarm = d.alarm;
  out_.latched = d.latched;
  out_.primary_joint = primary_joint_;
  for (int i = 0; i < kDof; ++i) {
    out_.physics_alarm(i) = d.physics(i) || d.physics_slow(i);
  }
  out_.command_alarm = d.command;
  out_.physics_statistic = physics_stat;
  out_.physics_slow_statistic = slow_stat;
  out_.command_statistic = command_stat;
  out_.limit = calibration_.limit;
  return out_;
}

}  // namespace ids_core