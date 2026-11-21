// =============================================================================
//  dual_ewma.cpp - the two averages and their statistics
// -----------------------------------------------------------------------------
//  Statistic of the difference channel: |fast - slow| / sigma_D(k).
//  Statistic of the slow channel:       |slow_live| / sigma_S(k).
//  sigma is computed exactly for the k-th sample, so the statistic has unit variance
//  from the very first window (the dissertation used the long-run value only).
// =============================================================================

#include "ids_core/dual_ewma.hpp"

#include <algorithm>
#include <cmath>

namespace ids_core {

// Smoothing factor from a time constant: tau = 0.0448 s at 100 Hz gives lambda = 0.2.
double lambdaFromTau(double dt, double tau) { return 1.0 - std::exp(-dt / tau); }

void DualEwma::configure(double dt, double tau_fast, double tau_slow) {
  lf_ = lambdaFromTau(dt, tau_fast);
  ls_ = lambdaFromTau(dt, tau_slow);
  reset();
}

void DualEwma::reset() {
  fast_.setZero();
  slow_.setZero();
  slow_live_.setZero();
  difference_.setZero();
  statistic_.setZero();
  slow_statistic_.setZero();
  history_head_ = 0;
  history_size_ = 0;
  pf2_ = 1.0;
  ps2_ = 1.0;
  pfs_ = 1.0;
}

// Variance of fast - slow for white noise after k samples: a + b - 2c, each term
// scaled by how far the averages have filled up (pf2_, ps2_, pfs_).
double DualEwma::sigmaDifference() const {
  const double a = lf_ / (2.0 - lf_);
  const double b = ls_ / (2.0 - ls_);
  const double c = lf_ * ls_ / (1.0 - (1.0 - lf_) * (1.0 - ls_));
  const double var = a * (1.0 - pf2_) + b * (1.0 - ps2_) - 2.0 * c * (1.0 - pfs_);
  return std::sqrt(std::max(var, 1e-12));
}

double DualEwma::sigmaSlow() const {
  const double b = ls_ / (2.0 - ls_);
  return std::sqrt(std::max(b * (1.0 - ps2_), 1e-12));
}

double DualEwma::asymptoticSigmaDifference() const {
  const double a = lf_ / (2.0 - lf_);
  const double b = ls_ / (2.0 - ls_);
  const double c = lf_ * ls_ / (1.0 - (1.0 - lf_) * (1.0 - ls_));
  return std::sqrt(a + b - 2.0 * c);
}

// One window: update both averages, store the baseline for rollback, recompute both statistics.
void DualEwma::update(const Vec7& x, bool freeze_slow) {
  fast_ = lf_ * x + (1.0 - lf_) * fast_;
  slow_live_ = ls_ * x + (1.0 - ls_) * slow_live_;
  // While an alarm is active the baseline stays still, so the attack is not absorbed into it.
  if (!freeze_slow) {
    history_[history_head_] = slow_;
    history_head_ = (history_head_ + 1) % kHistory;
    history_size_ = std::min(history_size_ + 1, kHistory);
    slow_ = ls_ * x + (1.0 - ls_) * slow_;
  }
  pf2_ *= (1.0 - lf_) * (1.0 - lf_);
  ps2_ *= (1.0 - ls_) * (1.0 - ls_);
  pfs_ *= (1.0 - lf_) * (1.0 - ls_);
  difference_ = fast_ - slow_;
  statistic_ = difference_.cwiseAbs() / sigmaDifference();
  slow_statistic_ = slow_live_.cwiseAbs() / sigmaSlow();
}

// At an alarm onset, return the baseline to where it was `steps` windows ago:
// the first windows of an attack must not count as normal (B23).
void DualEwma::rollbackBaseline(int steps) {
  if (steps <= 0 || history_size_ == 0) {
    return;
  }
  const std::size_t back = std::min<std::size_t>(static_cast<std::size_t>(steps), history_size_);
  slow_ = history_[(history_head_ + kHistory - back) % kHistory];
  difference_ = fast_ - slow_;
  statistic_ = difference_.cwiseAbs() / sigmaDifference();
}

}  // namespace ids_core
