#include <gtest/gtest.h>

#include "iiwa_sim/friction_model.hpp"

using iiwa_sim::ForceComposer;
using iiwa_sim::FrictionChannel;
using iiwa_sim::FrictionSchedule;

TEST(FrictionSchedule, HoldsTheInitialValueUntilTheDelay) {
  const FrictionSchedule s(30.0, {{4.0, 0.004, 8.0}});
  EXPECT_DOUBLE_EQ(s.coefficient(0, 0.0), 4.0);
  EXPECT_DOUBLE_EQ(s.coefficient(0, 29.999), 4.0);
}

TEST(FrictionSchedule, GrowsLinearlyAfterTheDelayAndSaturates) {
  const FrictionSchedule s(30.0, {{4.0, 0.004, 8.0}});
  EXPECT_NEAR(s.coefficient(0, 130.0), 4.4, 1e-12);
  EXPECT_NEAR(s.coefficient(0, 1030.0), 8.0, 1e-12);
  EXPECT_NEAR(s.coefficient(0, 5000.0), 8.0, 1e-12);
}

TEST(FrictionSchedule, ThesisProfileReproducesTheV1Drift) {
  const FrictionSchedule s(60.0, {{0.0, 0.0005, 15.0}});
  EXPECT_DOUBLE_EQ(s.coefficient(0, 60.0), 0.0);
  EXPECT_NEAR(s.coefficient(0, 940.0), 0.44, 1e-12);
}

TEST(FrictionSchedule, AMaximumBelowTheInitialValueNeverReducesIt) {
  const FrictionSchedule s(0.0, {{2.0, 0.1, 0.0}, {0.0, 0.0, 0.0}});
  EXPECT_DOUBLE_EQ(s.coefficient(0, 100.0), 2.0);
  EXPECT_DOUBLE_EQ(s.coefficient(1, 100.0), 0.0);
}

TEST(ForceComposer, AddsFrictionOnTopOfACommandRewrittenEveryStep) {
  ForceComposer c;
  EXPECT_DOUBLE_EQ(c.compose(5.0, -1.0), 4.0);
  EXPECT_DOUBLE_EQ(c.compose(5.0, -1.1), 3.9);
  EXPECT_DOUBLE_EQ(c.compose(-2.0, 0.5), -1.5);
}

TEST(ForceComposer, DoesNotAccumulateWhenNobodyRewritesTheCommand) {
  ForceComposer c;
  double cmd = c.compose(5.0, -1.0);
  for (int k = 0; k < 100; ++k) {
    cmd = c.compose(cmd, -1.0);
  }
  EXPECT_DOUBLE_EQ(cmd, 4.0);
  EXPECT_DOUBLE_EQ(c.compose(cmd, -2.0), 3.0);
}

TEST(ForceComposer, StartsFromZeroAfterThePhysicsClearsTheCommand) {
  ForceComposer c;
  c.compose(5.0, -1.0);
  EXPECT_DOUBLE_EQ(c.compose(0.0, -0.3), -0.3);
}