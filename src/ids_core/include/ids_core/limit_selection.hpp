// =============================================================================
//  limit_selection.hpp - choose alarm limits from a false-alarm budget
// -----------------------------------------------------------------------------
//  Instead of a hand-tuned threshold, the limit is the value that gives the target
//  number of false alarms per hour on pure noise, for this exact detector (time
//  constants, debounce, hysteresis).
// =============================================================================

#pragma once

#include <cstdint>
#include <vector>

namespace ids_core {

enum class StatisticKind { kDifference, kSlow };

struct AlarmModel {
  double rate_hz = 100.0;
  double tau_fast = 0.0448;
  double tau_slow = 1.995;
  int debounce = 5;
  int clear_count = 5;
  double hysteresis = 0.5;
  StatisticKind kind = StatisticKind::kDifference;
};

std::vector<double> falseAlarmRatesPerHour(const AlarmModel& model,
                                           const std::vector<double>& limits, double hours,
                                           std::uint64_t seed);

double selectLimit(const AlarmModel& model, double target_per_hour, double hours,
                   std::uint64_t seed);

}  // namespace ids_core
