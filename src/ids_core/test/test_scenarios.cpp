// =============================================================================
//  test_scenarios.cpp - end-to-end attack scenarios on the synthetic cell
// -----------------------------------------------------------------------------
//  The claims of the project, checked on the synthetic cell:
//  - wear alone raises no alarm;
//  - a 0.02 rad setpoint step on any joint is detected within 100 ms on that joint;
//  - a slow ramp is caught by physics alone;
//  - torque tampering after the controller is caught by the command check;
//  - the dissertation's design (commanded-torque input) is blind to a setpoint MitM;
//  - a payload handover inside the context guard raises no alarm.
// =============================================================================

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

#include "synthetic_cell.hpp"

using ids_core::Calibration;
using ids_core::EstimatorMode;
using ids_core::IdsConfig;
using ids_core::IdsPipeline;
using ids_core::InputSource;
using ids_core::Mask7;
using ids_core::Vec7;
using ids_core::test::Attack;
using ids_core::test::AttackKind;
using ids_core::test::CellConfig;
using ids_core::test::SyntheticCell;

namespace {

std::string urdf() {
  std::ifstream f(ids_core::test::urdfPath());
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

IdsConfig extended() { return IdsConfig{}; }

IdsConfig physicsOnly() {
  IdsConfig c;
  c.detector.use_command_check = false;
  return c;
}

IdsConfig thesis() {
  IdsConfig c;
  c.mode = EstimatorMode::kThesis;
  c.input_source = InputSource::kCommandedTorque;
  return c;
}

Calibration calibrate(const IdsConfig& cfg, double seconds, std::uint64_t seed) {
  IdsPipeline ids(cfg, urdf());
  CellConfig cc;
  cc.seed = seed;
  SyntheticCell cell(cc);
  ids.startCalibration();
  const long n = std::lround(seconds / cc.ids_dt);
  for (long k = 0; k < n; ++k) ids.process(cell.step(ids));
  return ids.finishCalibration();
}

const Calibration& extendedCalibration() {
  static const Calibration c = calibrate(extended(), 60.0, 100);
  return c;
}

const Calibration& physicsOnlyCalibration() {
  static const Calibration c = calibrate(physicsOnly(), 60.0, 100);
  return c;
}

const Calibration& thesisCalibration() {
  static const Calibration c = calibrate(thesis(), 60.0, 500);
  return c;
}

struct Outcome {
  double first_alarm = -1.0;
  int primary = -1;
  Mask7 physics = Mask7::Constant(false);
  Mask7 command = Mask7::Constant(false);
  bool alarm_before_attack = false;
  Vec7 friction_estimate = Vec7::Zero();
  Vec7 friction_true = Vec7::Zero();
};

Outcome run(const IdsConfig& cfg, const Calibration& cal, Attack attack, double duration,
            std::uint64_t seed, double friction_growth = 0.0) {
  IdsPipeline ids(cfg, urdf());
  ids.arm(cal);
  CellConfig cc;
  cc.seed = seed;
  cc.friction_growth_per_s = friction_growth;
  SyntheticCell cell(cc);
  cell.setAttack(attack);
  Outcome o;
  const long n = std::lround((attack.start + duration) / cc.ids_dt);
  for (long k = 0; k < n; ++k) {
    const auto m = cell.step(ids);
    const auto& out = ids.process(m);
    if (!out.valid || !out.alarm) continue;
    if (m.stamp < attack.start - 1e-9) o.alarm_before_attack = true;
    if (o.first_alarm < 0.0) {
      o.first_alarm = m.stamp - attack.start;
      o.primary = out.primary_joint;
    }
    for (int i = 0; i < 7; ++i) {
      o.physics(i) = o.physics(i) || out.physics_alarm(i);
      o.command(i) = o.command(i) || out.command_alarm(i);
    }
  }
  o.friction_estimate = ids.ekf().state().b;
  o.friction_true = cell.friction();
  return o;
}

Mask7 only(int joint) {
  Mask7 m = Mask7::Constant(false);
  m(joint) = true;
  return m;
}

}  // namespace

// Friction wear only: no alarm.
TEST(Scenario, NominalMotionWithFrictionWearRaisesNoAlarm) {
  Attack none;
  none.start = 120.0;
  const Outcome o = run(extended(), extendedCalibration(), none, 0.0, 200, 1e-3);
  EXPECT_LT(o.first_alarm, 0.0);
  EXPECT_FALSE(o.alarm_before_attack);
  EXPECT_GT(o.friction_true(0), 1.12 * extendedCalibration().friction_initial(0));
  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(o.friction_estimate(i), o.friction_true(i), 0.1 * o.friction_true(i))
        << "joint " << i;
  }
}

// Step on each of the 7 joints: detected in under 100 ms and attributed correctly.
TEST(Scenario, SetpointStepOnEveryJointIsDetectedWithin100msAndAttributed) {
  for (int j = 0; j < 7; ++j) {
    Attack a;
    a.kind = AttackKind::kSetpointStep;
    a.joint = j;
    a.magnitude = 0.02;
    a.start = 5.0;
    const Outcome o = run(extended(), extendedCalibration(), a, 1.0, 300 + j);
    EXPECT_FALSE(o.alarm_before_attack) << "joint " << j;
    EXPECT_GE(o.first_alarm, 0.0) << "joint " << j;
    EXPECT_LE(o.first_alarm, 0.1) << "joint " << j;
    EXPECT_EQ(o.primary, j);
    EXPECT_TRUE(o.physics(j)) << "joint " << j;
    EXPECT_EQ(o.command, only(j)) << "joint " << j;
  }
}

// An onset on a window boundary flags only the attacked joint (contrast: B4).
TEST(Scenario, WindowAlignedOnsetGivesExactPhysicsAttribution) {
  Attack a;
  a.kind = AttackKind::kTorqueBias;
  a.joint = 4;
  a.magnitude = 5.0;
  a.start = 4.999;
  const Outcome aligned = run(physicsOnly(), physicsOnlyCalibration(), a, 1.0, 304);
  EXPECT_EQ(aligned.physics, only(4));
  EXPECT_EQ(aligned.primary, 4);
  a.start = 5.004;
  const Outcome mid = run(physicsOnly(), physicsOnlyCalibration(), a, 1.0, 304);
  EXPECT_TRUE(mid.physics(4));
  EXPECT_EQ(mid.primary, 4);
}

// A slow ramp is caught without the command check.
TEST(Scenario, SlowSetpointRampIsCaughtByPhysicsAlone) {
  Attack a;
  a.kind = AttackKind::kSetpointRamp;
  a.joint = 3;
  a.magnitude = 2e-4;
  a.start = 10.0;
  const Outcome o = run(physicsOnly(), physicsOnlyCalibration(), a, 5.0, 400);
  EXPECT_FALSE(o.alarm_before_attack);
  EXPECT_GE(o.first_alarm, 0.0);
  EXPECT_LE(o.first_alarm, 2.0);
  EXPECT_EQ(o.primary, 3);
  EXPECT_EQ(o.physics, only(3));
}

// Torque changed after the controller is caught.
TEST(Scenario, TorqueTamperAfterTheControllerIsDetected) {
  Attack a;
  a.kind = AttackKind::kTorqueBias;
  a.joint = 5;
  a.magnitude = 2.0;
  a.start = 5.0;
  const Outcome o = run(extended(), extendedCalibration(), a, 1.0, 402);
  EXPECT_GE(o.first_alarm, 0.0);
  EXPECT_LE(o.first_alarm, 0.1);
  EXPECT_EQ(o.primary, 5);
  EXPECT_EQ(o.command, only(5));
}

// Thesis mode misses a setpoint MitM that v2 catches (finding M1).
TEST(Scenario, ThesisFaithfulTorqueInputIsBlindToASetpointMitm) {
  Attack a;
  a.kind = AttackKind::kSetpointStep;
  a.joint = 3;
  a.magnitude = 0.02;
  a.start = 5.0;
  const Outcome blind = run(thesis(), thesisCalibration(), a, 3.0, 501);
  EXPECT_LT(blind.first_alarm, 0.0);
  const Outcome seen = run(extended(), extendedCalibration(), a, 3.0, 501);
  EXPECT_GE(seen.first_alarm, 0.0);
  EXPECT_LE(seen.first_alarm, 0.1);
}

// Grab and release inside the guard: no alarm (B9, B10).
TEST(Scenario, PayloadHandoverInsideTheContextGuardRaisesNoAlarm) {
  const auto alarms = [](const IdsConfig& cfg, const Calibration& cal) {
    IdsPipeline ids(cfg, urdf());
    ids.arm(cal);
    CellConfig cc;
    cc.seed = 600;
    cc.context_on = 6.0;
    cc.attach_at = 6.02;
    cc.context_off = 10.0;
    cc.detach_at = 10.08;
    SyntheticCell cell(cc);
    int count = 0;
    for (int k = 0; k < 1400; ++k) {
      const auto& out = ids.process(cell.step(ids));
      if (out.valid && out.alarm) ++count;
    }
    return count;
  };
  EXPECT_EQ(alarms(extended(), extendedCalibration()), 0);
  IdsConfig unguarded = extended();
  unguarded.context_guard = 0.0;
  EXPECT_GT(alarms(unguarded, calibrate(unguarded, 60.0, 100)), 0);
}
