// =============================================================================
//  test_detector.cpp - alarm logic and limit selection
// -----------------------------------------------------------------------------
//  Debounce, hysteresis, latch, channel separation, and that the chosen limits really
//  give the target false-alarm rate.
// =============================================================================

#include <gtest/gtest.h>

#include "ids_core/detector.hpp"
#include "ids_core/limit_selection.hpp"

using ids_core::AlarmModel;
using ids_core::Detector;
using ids_core::DetectorConfig;
using ids_core::Vec7;

namespace {

Detector make(bool latch = true) {
  DetectorConfig c;
  c.latch = latch;
  Detector d;
  d.configure(c, 4.0, 5.0);
  return d;
}

Vec7 on(int joint, double value) {
  Vec7 v = Vec7::Zero();
  v(joint) = value;
  return v;
}

}  // namespace

// One or four windows over the limit are not an alarm; five are.
TEST(Detector, NeedsDebounceSamplesInARow) {
  Detector d = make();
  for (int k = 0; k < 4; ++k) {
    EXPECT_FALSE(d.update(on(2, 5.0), Vec7::Zero(), Vec7::Zero(), true, true).alarm);
  }
  EXPECT_FALSE(d.update(Vec7::Zero(), Vec7::Zero(), Vec7::Zero(), true, true).alarm);
  for (int k = 0; k < 4; ++k) d.update(on(2, 5.0), Vec7::Zero(), Vec7::Zero(), true, true);
  const auto& out = d.update(on(2, 5.0), Vec7::Zero(), Vec7::Zero(), true, true);
  EXPECT_TRUE(out.alarm);
  EXPECT_TRUE(out.onset);
  EXPECT_TRUE(out.physics(2));
  EXPECT_FALSE(out.physics(3));
}

// One onset per alarm; clears only below half the limit.
TEST(Detector, OnsetFiresOnceAndHysteresisClears) {
  Detector d = make(false);
  for (int k = 0; k < 5; ++k) d.update(on(0, 5.0), Vec7::Zero(), Vec7::Zero(), true, true);
  EXPECT_FALSE(d.update(on(0, 3.0), Vec7::Zero(), Vec7::Zero(), true, true).onset);
  EXPECT_TRUE(d.output().alarm);
  for (int k = 0; k < 4; ++k) d.update(on(0, 1.0), Vec7::Zero(), Vec7::Zero(), true, true);
  EXPECT_TRUE(d.output().alarm);
  EXPECT_FALSE(d.update(on(0, 1.0), Vec7::Zero(), Vec7::Zero(), true, true).alarm);
  EXPECT_FALSE(d.output().latched);
}

// A latched alarm stays until reset.
TEST(Detector, LatchHoldsUntilReset) {
  Detector d = make(true);
  for (int k = 0; k < 5; ++k) d.update(on(5, 9.0), Vec7::Zero(), Vec7::Zero(), true, true);
  for (int k = 0; k < 10; ++k) d.update(Vec7::Zero(), Vec7::Zero(), Vec7::Zero(), true, true);
  EXPECT_FALSE(d.output().alarm);
  EXPECT_TRUE(d.output().latched);
  d.reset();
  EXPECT_FALSE(d.output().latched);
}

// Channels are independent and can be switched off.
TEST(Detector, SlowAndCommandChannelsAreSeparateAndCanBeDisabled) {
  Detector d = make();
  for (int k = 0; k < 5; ++k) d.update(Vec7::Zero(), on(1, 6.0), on(4, 6.0), false, false);
  EXPECT_FALSE(d.output().alarm);
  for (int k = 0; k < 5; ++k) d.update(Vec7::Zero(), on(1, 6.0), on(4, 6.0), true, true);
  EXPECT_TRUE(d.output().physics_slow(1));
  EXPECT_TRUE(d.output().command(4));
}

// Higher limits and longer debounce mean fewer false alarms.
TEST(LimitSelection, RatesFallWithLimitAndDebounce) {
  AlarmModel m;
  const auto r5 = ids_core::falseAlarmRatesPerHour(m, {3.0, 3.5, 4.0}, 2.0, 1);
  EXPECT_GT(r5[0], r5[1]);
  EXPECT_GT(r5[1], r5[2]);
  m.debounce = 1;
  const auto r1 = ids_core::falseAlarmRatesPerHour(m, {3.0}, 2.0, 1);
  EXPECT_GT(r1[0], r5[0]);
}

// The chosen limit meets its target on new noise.
TEST(LimitSelection, SelectedLimitMeetsItsTargetOnFreshNoise) {
  AlarmModel m;
  const double target = 20.0;
  const double limit = ids_core::selectLimit(m, target, 5.0, 3);
  const double check = ids_core::falseAlarmRatesPerHour(m, {limit}, 10.0, 99)[0];
  EXPECT_GT(check, target / 2.0);
  EXPECT_LT(check, target * 2.0);
}

// Very small budgets give sensible, higher limits.
TEST(LimitSelection, TinyBudgetsExtrapolateToHigherLimits) {
  AlarmModel m;
  const double loose = ids_core::selectLimit(m, 10.0, 3.0, 5);
  const double strict = ids_core::selectLimit(m, 0.01, 3.0, 5);
  EXPECT_GT(strict, loose + 0.3);
  EXPECT_LT(strict, 8.0);
}
