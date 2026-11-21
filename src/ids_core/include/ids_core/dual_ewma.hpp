#pragma once

#include <array>
#include <cstddef>

#include "ids_core/types.hpp"

namespace ids_core {

double lambdaFromTau(double dt, double tau);

class DualEwma {
 public:
  static constexpr std::size_t kHistory = 128;

  void configure(double dt, double tau_fast, double tau_slow);
  void reset();
  void update(const Vec7& x, bool freeze_slow);
  void rollbackBaseline(int steps);

  const Vec7& fast() const { return fast_; }
  const Vec7& slow() const { return slow_; }
  const Vec7& slowLive() const { return slow_live_; }
  const Vec7& difference() const { return difference_; }
  const Vec7& statistic() const { return statistic_; }
  const Vec7& slowStatistic() const { return slow_statistic_; }
  double lambdaFast() const { return lf_; }
  double lambdaSlow() const { return ls_; }
  double sigmaDifference() const;
  double sigmaSlow() const;
  double asymptoticSigmaDifference() const;

 private:
  double lf_ = 0.2;
  double ls_ = 0.005;
  Vec7 fast_ = Vec7::Zero();
  Vec7 slow_ = Vec7::Zero();
  Vec7 slow_live_ = Vec7::Zero();
  Vec7 difference_ = Vec7::Zero();
  Vec7 statistic_ = Vec7::Zero();
  Vec7 slow_statistic_ = Vec7::Zero();
  std::array<Vec7, kHistory> history_{};
  std::size_t history_head_ = 0;
  std::size_t history_size_ = 0;
  double pf2_ = 1.0;
  double ps2_ = 1.0;
  double pfs_ = 1.0;
};

}  // namespace ids_core