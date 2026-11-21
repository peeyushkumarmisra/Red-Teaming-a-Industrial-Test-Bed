#include "ids_core/dual_ewma.hpp"

#include <algorithm>
#include <cmath>

namespace ids_core {

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

void DualEwma::update(const Vec7& x, bool freeze_slow) {
  fast_ = lf_ * x + (1.0 - lf_) * fast_;
  slow_live_ = ls_ * x + (1.0 - ls_) * slow_live_;
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