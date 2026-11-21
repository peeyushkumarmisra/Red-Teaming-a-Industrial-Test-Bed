// =============================================================================
//  test_closed_loop_model.cpp - the closed-loop prediction is right
// -----------------------------------------------------------------------------
//  Jacobians against finite differences, agreement with the control law, tracking,
//  and recovery of a known disturbance per joint.
// =============================================================================

#include <gtest/gtest.h>

#include "ids_core/closed_loop_model.hpp"
#include "test_helpers.hpp"

using ids_core::ClosedLoopModel;
using ids_core::InputSource;
using ids_core::Mat21;
using ids_core::Mat21x7;
using ids_core::ModelConfig;
using ids_core::ReferenceTrack;
using ids_core::State;
using iiwa_control_law::RobotModel;
using iiwa_control_law::Vec7;

namespace {

struct Fixture {
  RobotModel robot = RobotModel::fromUrdfFile(ids_core::test::urdfPath());
  ModelConfig config;
  ReferenceTrack track;
  Fixture() { ids_core::test::fillTrack(track, 5.0); }
};

State startState() {
  State x;
  const auto s = ids_core::test::sinusoidSample(1.0, 0);
  x.q = s.position + Vec7::Constant(0.003);
  x.v = s.velocity;
  x.b << 0.5, 0.4, 0.3, 0.3, 0.05, 0.03, 0.01;
  return x;
}

ids_core::Vec21 pack(const State& x) {
  ids_core::Vec21 v;
  v << x.q, x.v, x.b;
  return v;
}

State unpack(const ids_core::Vec21& v) {
  State x;
  x.q = v.segment<7>(0);
  x.v = v.segment<7>(7);
  x.b = v.segment<7>(14);
  return x;
}

void checkJacobians(InputSource source) {
  Fixture fx;
  ClosedLoopModel model(fx.robot, fx.config, source);
  const Vec7 applied = Vec7::Constant(3.0);
  State x = startState();
  Mat21 f;
  Mat21x7 g;
  model.propagate(x, 1.0, 10, fx.track, applied, &f, &g);
  const double eps = 1e-6;
  Mat21 nf;
  for (int j = 0; j < 21; ++j) {
    ids_core::Vec21 p = pack(startState()), n = pack(startState());
    p(j) += eps;
    n(j) -= eps;
    State xp = unpack(p), xn = unpack(n);
    model.propagate(xp, 1.0, 10, fx.track, applied, nullptr, nullptr);
    model.propagate(xn, 1.0, 10, fx.track, applied, nullptr, nullptr);
    nf.col(j) = (pack(xp) - pack(xn)) / (2 * eps);
  }
  Mat21x7 ng;
  for (int j = 0; j < 7; ++j) {
    Vec7 dp = Vec7::Zero(), dn = Vec7::Zero();
    dp(j) = eps;
    dn(j) = -eps;
    State xp = startState(), xn = startState();
    model.propagate(xp, 1.0, 10, fx.track, applied, nullptr, nullptr, &dp);
    model.propagate(xn, 1.0, 10, fx.track, applied, nullptr, nullptr, &dn);
    ng.col(j) = (pack(xp) - pack(xn)) / (2 * eps);
  }
  EXPECT_LT((f - nf).norm() / nf.norm(), 1e-4);
  EXPECT_LT((g - ng).norm() / ng.norm(), 1e-4);
}

}  // namespace

// F and G are correct in v2 mode.
TEST(ClosedLoopModel, TrustedReferenceJacobiansMatchFiniteDifference) {
  checkJacobians(InputSource::kTrustedReference);
}

// F and G are correct in thesis mode.
TEST(ClosedLoopModel, CommandedTorqueJacobiansMatchFiniteDifference) {
  checkJacobians(InputSource::kCommandedTorque);
}

// The command check uses exactly the controller's law.
TEST(ClosedLoopModel, NominalTorqueMatchesControlLaw) {
  Fixture fx;
  ClosedLoopModel model(fx.robot, fx.config, InputSource::kTrustedReference);
  const auto s = ids_core::test::sinusoidSample(2.0, 7);
  const Vec7 q = s.position + Vec7::Constant(0.01);
  const Vec7 v = s.velocity;
  const Vec7 expected = iiwa_control_law::computeTorque(
      fx.config.gains, q, v, iiwa_control_law::referenceAt(s, 2.004, fx.config.reference_hold),
      fx.robot.gravity(q));
  EXPECT_LT((model.nominalTorque(q, v, s, 2.004) - expected).norm(), 1e-12);
}

// The modelled arm follows the reference like the real one.
TEST(ClosedLoopModel, TracksReferenceInClosedLoop) {
  Fixture fx;
  ClosedLoopModel model(fx.robot, fx.config, InputSource::kTrustedReference);
  State x;
  const auto s0 = ids_core::test::sinusoidSample(0.0, 0);
  x.q = s0.position;
  x.v = s0.velocity;
  model.propagate(x, 0.0, 3000, fx.track, Vec7::Zero(), nullptr, nullptr);
  const auto s = ids_core::test::sinusoidSample(3.0, 0);
  EXPECT_LT((x.q - s.position).cwiseAbs().maxCoeff(), 0.02);
}

// Phi^-1 recovers a torque applied to one joint, on that joint.
TEST(ClosedLoopModel, DisturbanceInversionRecoversPerJointTorque) {
  Fixture fx;
  ClosedLoopModel model(fx.robot, fx.config, InputSource::kTrustedReference);
  for (int j = 0; j < 7; ++j) {
    Vec7 d = Vec7::Zero();
    d(j) = fx.config.gains.kp(j) * 0.02;
    State truth = startState();
    State pred = startState();
    Mat21 f;
    Mat21x7 g;
    model.propagate(truth, 1.0, 10, fx.track, Vec7::Zero(), nullptr, nullptr, &d);
    model.propagate(pred, 1.0, 10, fx.track, Vec7::Zero(), &f, &g);
    const iiwa_control_law::Mat7 phi = g.middleRows<7>(7);
    const Vec7 est = phi.partialPivLu().solve(truth.v - pred.v);
    EXPECT_NEAR(est(j), d(j), 0.02 * d(j)) << "joint " << j;
    Vec7 others = est;
    others(j) = 0.0;
    EXPECT_LT(others.cwiseAbs().maxCoeff(), 0.02 * d(j)) << "joint " << j;
  }
}