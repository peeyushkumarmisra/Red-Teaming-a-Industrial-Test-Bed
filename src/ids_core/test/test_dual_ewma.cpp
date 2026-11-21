// =============================================================================
//  test_dual_ewma.cpp - the two averages and their statistics
// -----------------------------------------------------------------------------
//  Time constants, exact variance, drift response, freeze and rollback.
// =============================================================================

#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "ids_core/dual_ewma.hpp"

using ids_core::DualEwma;
using ids_core::lambdaFromTau;
using ids_core::Vec7;

// tau values give the dissertation's lambda 0.2 and 0.005.
TEST(DualEwma, TimeConstantsReproduceThesisLambdas) {
  EXPECT_NEAR(lambdaFromTau(0.01, 0.0448), 0.2, 2e-3);
  EXPECT_NEAR(lambdaFromTau(0.01, 1.995), 0.005, 1e-4);
}

// Long-run sigma of fast - slow matches the formula.
TEST(DualEwma, AsymptoticSigmaOfDifferenceMatchesClosedForm) {
  DualEwma e;
  e.configure(0.01, 0.0448, 1.995);
  EXPECT_NEAR(e.asymptoticSigmaDifference(), 0.3222, 2e-3);
}

// Exact sigma: no false alarms right after a reset.
TEST(DualEwma, StatisticIsUnitVarianceOnWhiteNoiseFromTheFirstSample) {
  DualEwma e;
  e.configure(0.01, 0.0448, 1.995);
  std::mt19937 rng(3);
  std::normal_distribution<double> n(0.0, 1.0);
  double early = 0.0;
  double late = 0.0;
  const int runs = 4000;
  for (int r = 0; r < runs; ++r) {
    e.reset();
    for (int k = 0; k < 300; ++k) {
      Vec7 x;
      for (int i = 0; i < 7; ++i) x(i) = n(rng);
      e.update(x, false);
      if (k == 2) early += e.statistic().squaredNorm() / 7.0;
      if (k == 299) late += e.statistic().squaredNorm() / 7.0;
    }
  }
  EXPECT_NEAR(early / runs, 1.0, 0.05);
  EXPECT_NEAR(late / runs, 1.0, 0.05);
}

// A linear drift leaves a constant, predictable offset (wear stays below the limit).
TEST(DualEwma, LinearDriftSettlesAt195TimesSlope) {
  DualEwma e;
  e.configure(0.01, 0.0448, 1.995);
  const double slope = 1e-3;
  for (int k = 0; k < 20000; ++k) {
    e.update(Vec7::Constant(slope * k), false);
  }
  const double expected = slope * ((1.0 - e.lambdaSlow()) / e.lambdaSlow() -
                                   (1.0 - e.lambdaFast()) / e.lambdaFast());
  EXPECT_NEAR(e.difference()(0), expected, 1e-6);
  EXPECT_NEAR(expected, 0.195, 3e-3);
}

// Frozen baseline stays put.
TEST(DualEwma, FreezeHoldsSlowAverage) {
  DualEwma e;
  e.configure(0.01, 0.0448, 1.995);
  for (int k = 0; k < 100; ++k) e.update(Vec7::Constant(1.0), false);
  const double held = e.slow()(0);
  for (int k = 0; k < 100; ++k) e.update(Vec7::Constant(5.0), true);
  EXPECT_DOUBLE_EQ(e.slow()(0), held);
  EXPECT_GT(e.fast()(0), 4.9);
}

// The slow detector keeps working during a freeze (B23).
TEST(DualEwma, SlowStatisticKeepsFollowingTheDataWhileTheBaselineIsFrozen) {
  DualEwma e;
  e.configure(0.01, 0.0448, 1.995);
  for (int k = 0; k < 600; ++k) e.update(Vec7::Zero(), false);
  for (int k = 0; k < 300; ++k) e.update(Vec7::Constant(1.0), k >= 5);
  const double raised = e.slowStatistic()(0);
  EXPECT_GT(raised, 10.0);
  for (int k = 0; k < 1500; ++k) e.update(Vec7::Zero(), true);
  EXPECT_LT(e.slowStatistic()(0), 0.05 * raised);
  EXPECT_NEAR(e.slowLive()(0), 0.0, 1e-3);
  EXPECT_GT(e.slow()(0), 0.0);
}

// An abrupt change appears in the fast average first.
TEST(DualEwma, StepIsSeenByFastBeforeSlow) {
  DualEwma e;
  e.configure(0.01, 0.0448, 1.995);
  for (int k = 0; k < 500; ++k) e.update(Vec7::Zero(), false);
  for (int k = 0; k < 5; ++k) e.update(Vec7::Constant(2.0), false);
  EXPECT_GT(e.statistic()(0), 3.0);
}

// Rollback removes the start of an attack from the baseline.
TEST(DualEwma, RollbackRestoresTheBaselineFromBeforeTheExcursion) {
  DualEwma e;
  e.configure(0.01, 0.0448, 1.995);
  for (int k = 0; k < 600; ++k) e.update(Vec7::Constant(0.2), false);
  const double before = e.slow()(0);
  for (int k = 0; k < 5; ++k) e.update(Vec7::Constant(50.0), false);
  EXPECT_GT(e.slow()(0), before + 1.0);
  e.rollbackBaseline(5);
  EXPECT_DOUBLE_EQ(e.slow()(0), before);
  for (int k = 0; k < 50; ++k) e.update(Vec7::Constant(0.2), true);
  EXPECT_LT(e.statistic()(0), 1.0);
  e.rollbackBaseline(1000);
  EXPECT_LE(e.slow()(0), before);
  EXPECT_GT(e.slow()(0), 0.0);
}
