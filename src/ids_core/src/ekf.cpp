#include "ids_core/ekf.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Cholesky>
#include <Eigen/LU>

namespace ids_core {

Ekf::Ekf(ClosedLoopModel& model, const EkfConfig& config, EstimatorMode mode)
    : model_(model), config_(config), mode_(mode) {
  q_diag_.segment<kDof>(0).setConstant(config_.q_position);
  q_diag_.segment<kDof>(kDof).setConstant(config_.q_velocity);
  q_diag_.segment<kDof>(2 * kDof).setConstant(mode_ == EstimatorMode::kExtended ? config_.q_friction
                                                                              : 0.0);
  r_diag_.head<kDof>().setConstant(config_.r_position);
  r_diag_.tail<kDof>().setConstant(config_.r_velocity);
  f_.setIdentity();
  g_.setZero();
  initialize(Vec7::Zero(), Vec7::Zero(), Vec7::Zero());
}

void Ekf::initialize(const Vec7& q, const Vec7& v, const Vec7& b) {
  x_.q = q;
  x_.v = v;
  x_.b = mode_ == EstimatorMode::kExtended ? b : Vec7::Zero();
  p_.setZero();
  p_.diagonal().segment<kDof>(0).setConstant(config_.p0_position);
  p_.diagonal().segment<kDof>(kDof).setConstant(config_.p0_velocity);
  p_.diagonal().segment<kDof>(2 * kDof).setConstant(
      mode_ == EstimatorMode::kExtended ? config_.p0_friction : 0.0);
  history_head_ = 0;
  history_size_ = 0;
  out_ = EkfOutput{};
  last_q_ = q;
  last_v_ = v;
  anchor_ = x_;
  window_start_ = x_;
  track_ = nullptr;
}

void Ekf::resync(const Vec7& q, const Vec7& v) {
  x_.q = q;
  x_.v = v;
  const Mat7 pb = p_.bottomRightCorner<kDof, kDof>();
  p_.setZero();
  p_.diagonal().segment<kDof>(0).setConstant(config_.p0_position);
  p_.diagonal().segment<kDof>(kDof).setConstant(config_.p0_velocity);
  p_.bottomRightCorner<kDof, kDof>() = pb;
  last_q_ = q;
  last_v_ = v;
  anchor_ = x_;
  window_start_ = x_;
  track_ = nullptr;
}

void Ekf::predict(double t0, int steps, const ReferenceTrack& track, const Vec7& applied_torque,
                  bool freeze_friction) {
  window_start_.q = last_q_;
  window_start_.v = last_v_;
  window_start_.b = x_.b;
  track_ = &track;
  window_t0_ = t0;
  window_steps_ = steps;
  window_torque_ = applied_torque;
  anchor_ = window_start_;
  model_.propagate(anchor_, t0, steps, track, applied_torque, nullptr, nullptr);
  model_.propagate(x_, t0, steps, track, applied_torque, &f_, &g_);
  p_ = f_ * p_ * f_.transpose();
  Vec21 q = q_diag_ * static_cast<double>(steps);
  if (freeze_friction) {
    q.segment<kDof>(2 * kDof).setZero();
  }
  p_.diagonal() += q;
}

const EkfOutput& Ekf::update(const Vec7& q_measured, const Vec7& v_measured,
                             bool allow_friction_update) {
  Vec14 r;
  r.head<kDof>() = q_measured - x_.q;
  r.tail<kDof>() = v_measured - x_.v;
  Mat14 s = p_.topLeftCorner<kMeas, kMeas>();
  s.diagonal() += r_diag_;
  const Eigen::LLT<Mat14> llt(s);
  const Mat21x14 pht = p_.leftCols<kMeas>();
  Mat21x14 k = llt.solve(pht.transpose()).transpose();
  const double nis = r.dot(llt.solve(r));
  const bool friction = mode_ == EstimatorMode::kExtended && allow_friction_update &&
                        nis <= config_.nis_gate;
  if (!friction) {
    k.bottomRows<kDof>().setZero();
  }
  const Vec21 dx = k * r;
  x_.q += dx.segment<kDof>(0);
  x_.v += dx.segment<kDof>(kDof);
  x_.b += dx.segment<kDof>(2 * kDof);
  x_.b = x_.b.cwiseMax(0.0).cwiseMin(config_.friction_max);
  Mat21 ikh = Mat21::Identity();
  ikh.leftCols<kMeas>() -= k;
  Mat21 kr = Mat21::Zero();
  kr.noalias() = k * r_diag_.asDiagonal() * k.transpose();
  p_ = ikh * p_ * ikh.transpose() + kr;
  p_ = 0.5 * (p_ + p_.transpose()).eval();

  friction_history_[history_head_] = x_.b;
  history_head_ = (history_head_ + 1) % kHistory;
  history_size_ = std::min(history_size_ + 1, kHistory);

  const Mat7 phi = g_.middleRows<kDof>(kDof);
  const Eigen::PartialPivLU<Mat7> lu(phi);
  const Mat7 phi_inv = lu.inverse();
  const Mat7 f_vq = f_.block<kDof, kDof>(kDof, 0);
  const Mat7 f_vv = f_.block<kDof, kDof>(kDof, kDof);
  Mat7 residual_cov = config_.r_position * (f_vq * f_vq.transpose()) +
                      config_.r_velocity * (f_vv * f_vv.transpose());
  residual_cov.diagonal().array() += config_.r_velocity;
  const Mat7 cov_d = phi_inv * residual_cov * phi_inv.transpose();
  out_.innovation = r;
  out_.anchored_residual.head<kDof>() = q_measured - anchor_.q;
  out_.anchored_residual.tail<kDof>() = v_measured - anchor_.v;
  out_.innovation_covariance = s;
  out_.nis = nis;
  out_.disturbance = phi_inv * out_.anchored_residual.tail<kDof>();
  if (track_ != nullptr) {
    for (int it = 0; it < config_.refine_iterations; ++it) {
      State probe = window_start_;
      model_.propagate(probe, window_t0_, window_steps_, *track_, window_torque_, nullptr, nullptr,
                       &out_.disturbance);
      out_.disturbance += phi_inv * (v_measured - probe.v);
    }
  }
  out_.disturbance_std = cov_d.diagonal().cwiseMax(1e-24).cwiseSqrt();
  out_.friction_updated = friction;
  last_q_ = q_measured;
  last_v_ = v_measured;
  return out_;
}

void Ekf::rollbackFriction(int steps) {
  if (mode_ != EstimatorMode::kExtended || history_size_ == 0 || steps <= 0) {
    return;
  }
  const std::size_t back =
      std::min<std::size_t>(static_cast<std::size_t>(steps), history_size_ - 1);
  x_.b = friction_history_[(history_head_ + kHistory - 1 - back) % kHistory];
}

}  // namespace ids_core