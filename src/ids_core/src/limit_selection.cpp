#include "ids_core/limit_selection.hpp"

#include <algorithm>
#include <cmath>
#include <random>

#include "ids_core/dual_ewma.hpp"

namespace ids_core {

std::vector<double> falseAlarmRatesPerHour(const AlarmModel& model,
                                           const std::vector<double>& limits, double hours,
                                           std::uint64_t seed) {
  const double dt = 1.0 / model.rate_hz;
  const double lf = lambdaFromTau(dt, model.tau_fast);
  const double ls = lambdaFromTau(dt, model.tau_slow);
  const double a = lf / (2.0 - lf);
  const double b = ls / (2.0 - ls);
  const double c = lf * ls / (1.0 - (1.0 - lf) * (1.0 - ls));
  const double sigma =
      model.kind == StatisticKind::kDifference ? std::sqrt(a + b - 2.0 * c) : std::sqrt(b);
  const std::size_t n = limits.size();
  std::vector<int> over(n, 0), under(n, 0), episodes(n, 0);
  std::vector<bool> active(n, false);
  std::mt19937_64 rng(seed);
  std::normal_distribution<double> normal(0.0, 1.0);
  const auto samples = static_cast<long>(hours * 3600.0 * model.rate_hz);
  const long warmup = static_cast<long>(20.0 * model.tau_slow * model.rate_hz);
  double zf = 0.0;
  double zs = 0.0;
  for (long k = -warmup; k < samples; ++k) {
    const double x = normal(rng);
    zf = lf * x + (1.0 - lf) * zf;
    zs = ls * x + (1.0 - ls) * zs;
    if (k < 0) {
      continue;
    }
    const double stat =
        std::abs(model.kind == StatisticKind::kDifference ? zf - zs : zs) / sigma;
    for (std::size_t i = 0; i < n; ++i) {
      if (!active[i]) {
        over[i] = stat > limits[i] ? over[i] + 1 : 0;
        if (over[i] >= model.debounce) {
          active[i] = true;
          under[i] = 0;
          ++episodes[i];
        }
      } else {
        under[i] = stat < model.hysteresis * limits[i] ? under[i] + 1 : 0;
        if (under[i] >= model.clear_count) {
          active[i] = false;
          over[i] = 0;
        }
      }
    }
  }
  std::vector<double> rates(n);
  for (std::size_t i = 0; i < n; ++i) {
    rates[i] = episodes[i] / hours;
  }
  return rates;
}

double selectLimit(const AlarmModel& model, double target_per_hour, double hours,
                   std::uint64_t seed) {
  std::vector<double> limits;
  for (double l = 2.0; l <= 6.0 + 1e-9; l += 0.25) {
    limits.push_back(l);
  }
  const std::vector<double> rates = falseAlarmRatesPerHour(model, limits, hours, seed);
  for (std::size_t i = 0; i + 1 < limits.size(); ++i) {
    if (rates[i] >= target_per_hour && rates[i + 1] < target_per_hour && rates[i + 1] > 0.0) {
      const double t = (std::log(rates[i]) - std::log(target_per_hour)) /
                       (std::log(rates[i]) - std::log(rates[i + 1]));
      return limits[i] + t * (limits[i + 1] - limits[i]);
    }
  }
  double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
  int m = 0;
  for (std::size_t i = 0; i < limits.size(); ++i) {
    if (rates[i] * hours >= 10.0) {
      const double x = limits[i] * limits[i];
      const double y = std::log(rates[i]);
      sx += x;
      sy += y;
      sxx += x * x;
      sxy += x * y;
      ++m;
    }
  }
  if (m < 2) {
    return limits.back();
  }
  const double slope = (m * sxy - sx * sy) / (m * sxx - sx * sx);
  const double intercept = (sy - slope * sx) / m;
  if (slope >= 0.0) {
    return limits.back();
  }
  const double l2 = (std::log(target_per_hour) - intercept) / slope;
  return std::clamp(std::sqrt(std::max(l2, 0.0)), limits.front(), 8.0);
}

}  // namespace ids_core