// =============================================================================
//  ekf.hpp - extended Kalman filter over one 10 ms window
// -----------------------------------------------------------------------------
//  Tracks the arm's state (and friction in extended mode) and estimates, for every
//  window, the extra torque per joint that would explain what the encoders saw.
//  That torque estimate (the "disturbance") is what the detector watches.
// =============================================================================

#pragma once

#include <array>
#include <cstddef>

#include "ids_core/closed_loop_model.hpp"
#include "ids_core/config.hpp"
#include "ids_core/types.hpp"

namespace ids_core {

// Everything one filter step produced.
struct EkfOutput {
  Vec14 innovation = Vec14::Zero();
  Vec14 anchored_residual = Vec14::Zero();
  Mat14 innovation_covariance = Mat14::Identity();
  double nis = 0.0;
  // Estimated extra torque per joint over the window (N*m).
  Vec7 disturbance = Vec7::Zero();
  // Its expected noise, used to put every joint on the same scale.
  Vec7 disturbance_std = Vec7::Ones();
  bool friction_updated = false;
};

class Ekf {
 public:
  static constexpr std::size_t kHistory = 256;

  Ekf(ClosedLoopModel& model, const EkfConfig& config, EstimatorMode mode);

  void initialize(const Vec7& q, const Vec7& v, const Vec7& b);
  // Jump to the measured state without detecting (halted windows, payload handover).
  // Friction and its uncertainty are kept.
  void resync(const Vec7& q, const Vec7& v);
  void predict(double t0, int steps, const ReferenceTrack& track, const Vec7& applied_torque,
               bool freeze_friction);
  const EkfOutput& update(const Vec7& q_measured, const Vec7& v_measured,
                          bool allow_friction_update);
  // Restore the friction estimate from `steps` windows ago (alarm onset).
  void rollbackFriction(int steps);

  const State& state() const { return x_; }
  const Mat21& covariance() const { return p_; }
  const Mat21& transition() const { return f_; }
  const Mat21x7& disturbanceMap() const { return g_; }
  const EkfOutput& output() const { return out_; }
  EstimatorMode mode() const { return mode_; }

 private:
  ClosedLoopModel& model_;
  EkfConfig config_;
  EstimatorMode mode_;
  State x_;
  Mat21 p_;
  Mat21 f_;
  Mat21x7 g_;
  Vec21 q_diag_;
  Vec14 r_diag_;
  EkfOutput out_;
  State anchor_;
  State window_start_;
  const ReferenceTrack* track_ = nullptr;
  double window_t0_ = 0.0;
  int window_steps_ = 0;
  Vec7 window_torque_ = Vec7::Zero();
  Vec7 last_q_;
  Vec7 last_v_;
  // Last 256 friction estimates for rollback; fixed size, no allocation.
  std::array<Vec7, kHistory> friction_history_{};
  std::size_t history_head_ = 0;
  std::size_t history_size_ = 0;
};

}  // namespace ids_core
