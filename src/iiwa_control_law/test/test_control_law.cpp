// =============================================================================
//  test_control_law.cpp - unit tests of the shared control law
// -----------------------------------------------------------------------------
//  Proves the properties the IDS relies on: the hold between samples, the torque
//  formula, the clamp, and that a setpoint attack shows up as Kp * gamma on the
//  attacked joint only.
// =============================================================================

#include <gtest/gtest.h>

#include "iiwa_control_law/control_law.hpp"

using iiwa_control_law::computeTorque;
using iiwa_control_law::Gains;
using iiwa_control_law::holdTorque;
using iiwa_control_law::Mask7;
using iiwa_control_law::ReferencePoint;
using iiwa_control_law::ReferenceSample;
using iiwa_control_law::referenceAt;
using iiwa_control_law::Vec7;

// Between samples the reference moves along its velocity.
TEST(ControlLaw, FirstOrderHoldExtrapolatesWithVelocity) {
  ReferenceSample s;
  s.stamp = 1.0;
  s.position = Vec7::Constant(0.5);
  s.velocity = Vec7::Constant(0.2);
  const ReferencePoint r = referenceAt(s, 1.005, 0.05);
  EXPECT_NEAR(r.position(0), 0.501, 1e-12);
  EXPECT_NEAR(r.velocity(3), 0.2, 1e-12);
}

// A tick before the sample's stamp uses the sample unchanged.
TEST(ControlLaw, ReferenceBeforeStampIsSampleItself) {
  ReferenceSample s;
  s.stamp = 2.0;
  s.position = Vec7::Constant(-0.3);
  s.velocity = Vec7::Constant(1.0);
  const ReferencePoint r = referenceAt(s, 1.9, 0.05);
  EXPECT_DOUBLE_EQ(r.position(2), -0.3);
}

// A late reference stops after max_hold instead of running away.
TEST(ControlLaw, StaleReferenceHoldsWithZeroVelocity) {
  ReferenceSample s;
  s.stamp = 0.0;
  s.position = Vec7::Zero();
  s.velocity = Vec7::Constant(1.0);
  const ReferencePoint r = referenceAt(s, 10.0, 0.05);
  EXPECT_NEAR(r.position(6), 0.05, 1e-12);
  EXPECT_DOUBLE_EQ(r.velocity.norm(), 0.0);
}

// The torque is exactly PD plus gravity.
TEST(ControlLaw, TorqueIsPdPlusGravity) {
  const Gains g = Gains::iiwaDefault();
  const Vec7 q = Vec7::Zero();
  const Vec7 v = Vec7::Zero();
  Vec7 grav = Vec7::Zero();
  grav(1) = 10.0;
  ReferencePoint r{Vec7::Constant(0.01), Vec7::Zero()};
  const Vec7 tau = computeTorque(g, q, v, r, grav);
  for (int i = 0; i < 7; ++i) {
    EXPECT_NEAR(tau(i), g.kp(i) * 0.01 + grav(i), 1e-12);
  }
}

// Torques are clamped to the limits and the clamp is reported.
TEST(ControlLaw, TorqueIsClampedAndMaskReportsSaturation) {
  const Gains g = Gains::iiwaDefault();
  ReferencePoint r{Vec7::Constant(10.0), Vec7::Zero()};
  Mask7 sat;
  const Vec7 tau = computeTorque(g, Vec7::Zero(), Vec7::Zero(), r, Vec7::Zero(), &sat);
  for (int i = 0; i < 7; ++i) {
    EXPECT_DOUBLE_EQ(tau(i), g.effort_limit(i));
    EXPECT_TRUE(sat(i));
  }
}

// The physical signature of a setpoint attack: Kp * gamma on one joint.
TEST(ControlLaw, AttackOnSetpointShiftsTorqueByKpGamma) {
  const Gains g = Gains::iiwaDefault();
  const Vec7 q = Vec7::Constant(0.1);
  const Vec7 v = Vec7::Constant(0.05);
  ReferencePoint trusted{Vec7::Constant(0.12), Vec7::Constant(0.04)};
  ReferencePoint attacked = trusted;
  attacked.position(3) += 0.02;
  const Vec7 grav = Vec7::Constant(1.0);
  const Vec7 diff = computeTorque(g, q, v, attacked, grav) - computeTorque(g, q, v, trusted, grav);
  for (int i = 0; i < 7; ++i) {
    EXPECT_NEAR(diff(i), i == 3 ? g.kp(3) * 0.02 : 0.0, 1e-9);
  }
}

// Holding a position damps any motion.
TEST(ControlLaw, HoldTorqueDampsMotion) {
  const Gains g = Gains::iiwaDefault();
  const Vec7 tau = holdTorque(g, Vec7::Zero(), Vec7::Constant(0.1), Vec7::Zero(), Vec7::Zero());
  for (int i = 0; i < 7; ++i) {
    EXPECT_NEAR(tau(i), -g.kd(i) * 0.1, 1e-12);
  }
}
