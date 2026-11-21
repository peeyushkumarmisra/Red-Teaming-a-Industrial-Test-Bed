// =============================================================================
//  test_ekf.cpp - the Kalman filter behaves
// -----------------------------------------------------------------------------
//  Disturbance on the right joint, friction learning, gating, rollback, re-sync and the
//  refinement step.
// =============================================================================

#include <gtest/gtest.h>

#include <random>

#include <Eigen/Eigenvalues>

#include "ids_core/ekf.hpp"
#include "test_helpers.hpp"

using ids_core::ClosedLoopModel;
using ids_core::Ekf;
using ids_core::EkfConfig;
using ids_core::EstimatorMode;
using ids_core::InputSource;
using ids_core::Mat21;
using ids_core::ModelConfig;
using ids_core::ReferenceTrack;
using ids_core::State;
using iiwa_control_law::RobotModel;
using iiwa_control_law::Vec7;

namespace {

// A small plant + model pair for filter tests.
struct Bench {
  RobotModel truth_robot = RobotModel::fromUrdfFile(ids_core::test::urdfPath());
  RobotModel ids_robot = RobotModel::fromUrdfFile(ids_core::test::urdfPath());
  ModelConfig config;
  ReferenceTrack track;
  ids_core::test::Feeder feeder{track};
  ClosedLoopModel truth_model{truth_robot, config, InputSource::kTrustedReference};
  ClosedLoopModel ids_model{ids_robot, config, InputSource::kTrustedReference};
  State truth;
  std::mt19937 rng{42};
  double vel_noise = 0.0;

  Bench() {
    feeder.until(0.0);
    const auto s = ids_core::test::sinusoidSample(0.0, 0);
    truth.q = s.position;
    truth.v = s.velocity;
  }

  void advance(double t0, const Vec7* disturbance) {
    feeder.until(t0 + 0.01);
    truth_model.propagate(truth, t0, 10, track, Vec7::Zero(), nullptr, nullptr, disturbance);
  }

  Vec7 measuredVelocity() {
    std::normal_distribution<double> n(0.0, vel_noise);
    Vec7 v = truth.v;
    if (vel_noise > 0.0) {
      for (int i = 0; i < 7; ++i) v(i) += n(rng);
    }
    return v;
  }
};

}  // namespace

// With a perfect model the filter sees nothing.
TEST(Ekf, NoiseFreeNominalHasNearZeroInnovation) {
  Bench b;
  Ekf ekf(b.ids_model, EkfConfig{}, EstimatorMode::kThesis);
  ekf.initialize(b.truth.q, b.truth.v, Vec7::Zero());
  double max_d = 0.0;
  for (int k = 0; k < 300; ++k) {
    const double t0 = 0.01 * k;
    b.advance(t0, nullptr);
    ekf.predict(t0, 10, b.track, Vec7::Zero(), false);
    const auto& out = ekf.update(b.truth.q, b.truth.v, true);
    max_d = std::max(max_d, out.disturbance.cwiseAbs().maxCoeff());
  }
  EXPECT_LT(max_d, 1e-3);
}

// A 12 N*m disturbance appears on its joint and barely leaks elsewhere (B1).
TEST(Ekf, ConstantDisturbanceIsRecoveredOnItsJoint) {
  Bench b;
  Ekf ekf(b.ids_model, EkfConfig{}, EstimatorMode::kThesis);
  ekf.initialize(b.truth.q, b.truth.v, Vec7::Zero());
  Vec7 d = Vec7::Zero();
  d(3) = 12.0;
  Vec7 last = Vec7::Zero();
  for (int k = 0; k < 200; ++k) {
    const double t0 = 0.01 * k;
    b.advance(t0, k >= 100 ? &d : nullptr);
    ekf.predict(t0, 10, b.track, Vec7::Zero(), false);
    last = ekf.update(b.truth.q, b.truth.v, true).disturbance;
  }
  EXPECT_NEAR(last(3), 12.0, 1.0);
  Vec7 others = last;
  others(3) = 0.0;
  EXPECT_LT(others.cwiseAbs().maxCoeff(), 1.0);
}

// Extended mode learns the true friction.
TEST(Ekf, ExtendedModeLearnsViscousFriction) {
  Bench b;
  EkfConfig cfg;
  cfg.q_friction = 1e-6;
  Ekf ekf(b.ids_model, cfg, EstimatorMode::kExtended);
  ekf.initialize(b.truth.q, b.truth.v, Vec7::Zero());
  Vec7 b_true;
  b_true << 4.0, 4.0, 2.0, 2.0, 0.2, 0.1, 0.05;
  b.vel_noise = 1e-3;
  for (int k = 0; k < 6000; ++k) {
    const double t0 = 0.01 * k;
    b.truth.b = b_true;
    b.advance(t0, nullptr);
    ekf.predict(t0, 10, b.track, Vec7::Zero(), false);
    ekf.update(b.truth.q, b.measuredVelocity(), true);
  }
  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(ekf.state().b(i), b_true(i), 0.25 * b_true(i)) << "joint " << i;
  }
}

// Thesis mode never learns friction (as in the dissertation).
TEST(Ekf, ThesisModeKeepsFrictionAtZero) {
  Bench b;
  Ekf ekf(b.ids_model, EkfConfig{}, EstimatorMode::kThesis);
  ekf.initialize(b.truth.q, b.truth.v, Vec7::Constant(3.0));
  b.truth.b = Vec7::Constant(2.0);
  for (int k = 0; k < 200; ++k) {
    b.advance(0.01 * k, nullptr);
    ekf.predict(0.01 * k, 10, b.track, Vec7::Zero(), false);
    ekf.update(b.truth.q, b.truth.v, true);
  }
  EXPECT_DOUBLE_EQ(ekf.state().b.norm(), 0.0);
}

// The covariance stays valid over many steps.
TEST(Ekf, CovarianceStaysSymmetricPositiveDefinite) {
  Bench b;
  b.vel_noise = 1e-3;
  Ekf ekf(b.ids_model, EkfConfig{}, EstimatorMode::kExtended);
  ekf.initialize(b.truth.q, b.truth.v, Vec7::Zero());
  for (int k = 0; k < 2000; ++k) {
    b.advance(0.01 * k, nullptr);
    ekf.predict(0.01 * k, 10, b.track, Vec7::Zero(), false);
    ekf.update(b.truth.q, b.measuredVelocity(), true);
  }
  const Mat21& p = ekf.covariance();
  EXPECT_LT((p - p.transpose()).norm(), 1e-12);
  Eigen::SelfAdjointEigenSolver<Mat21> es(p);
  EXPECT_GT(es.eigenvalues().minCoeff(), 0.0);
}

// A surprising window does not change friction.
TEST(Ekf, NisGateBlocksFrictionUpdatesOnOutliers) {
  Bench b;
  Ekf ekf(b.ids_model, EkfConfig{}, EstimatorMode::kExtended);
  ekf.initialize(b.truth.q, b.truth.v, Vec7::Zero());
  b.advance(0.0, nullptr);
  ekf.predict(0.0, 10, b.track, Vec7::Zero(), false);
  const auto& out = ekf.update(b.truth.q, b.truth.v + Vec7::Constant(1.0), true);
  EXPECT_GT(out.nis, EkfConfig{}.nis_gate);
  EXPECT_FALSE(out.friction_updated);
}

// Rollback restores the earlier estimate exactly.
TEST(Ekf, RollbackRestoresEarlierFriction) {
  Bench b;
  EkfConfig cfg;
  cfg.q_friction = 1e-6;
  Ekf ekf(b.ids_model, cfg, EstimatorMode::kExtended);
  ekf.initialize(b.truth.q, b.truth.v, Vec7::Zero());
  b.truth.b = Vec7::Constant(3.0);
  Vec7 at_100 = Vec7::Zero();
  for (int k = 0; k < 150; ++k) {
    b.advance(0.01 * k, nullptr);
    ekf.predict(0.01 * k, 10, b.track, Vec7::Zero(), false);
    ekf.update(b.truth.q, b.truth.v, true);
    if (k == 99) at_100 = ekf.state().b;
  }
  ekf.rollbackFriction(50);
  EXPECT_LT((ekf.state().b - at_100).norm(), 1e-12);
}

// Re-sync keeps what was learned about friction.
TEST(Ekf, ResyncMovesTheStateButKeepsFrictionAndItsCovariance) {
  Bench b;
  EkfConfig cfg;
  cfg.q_friction = 1e-6;
  Ekf ekf(b.ids_model, cfg, EstimatorMode::kExtended);
  ekf.initialize(b.truth.q, b.truth.v, Vec7::Zero());
  b.truth.b = Vec7::Constant(3.0);
  for (int k = 0; k < 150; ++k) {
    b.advance(0.01 * k, nullptr);
    ekf.predict(0.01 * k, 10, b.track, Vec7::Zero(), false);
    ekf.update(b.truth.q, b.truth.v, true);
  }
  const Vec7 friction = ekf.state().b;
  const iiwa_control_law::Mat7 pb = ekf.covariance().bottomRightCorner<7, 7>();
  const Vec7 q = b.truth.q + Vec7::Constant(0.1);
  const Vec7 v = Vec7::Constant(-0.2);
  ekf.resync(q, v);
  EXPECT_LT((ekf.state().q - q).norm(), 1e-15);
  EXPECT_LT((ekf.state().v - v).norm(), 1e-15);
  EXPECT_LT((ekf.state().b - friction).norm(), 1e-15);
  EXPECT_LT((ekf.covariance().bottomRightCorner<7, 7>() - pb).norm(), 1e-15);
  EXPECT_DOUBLE_EQ((ekf.covariance().block<7, 7>(0, 14).norm()), 0.0);
  EXPECT_DOUBLE_EQ(ekf.covariance()(0, 0), cfg.p0_position);
}

// Refinement removes cross-talk from large, fast disturbances (B3).
TEST(Ekf, RefinementRemovesSecondOrderCrossTalkFromFastTransients) {
  auto leak = [](int iterations) {
    Bench b;
    EkfConfig cfg;
    cfg.refine_iterations = iterations;
    Ekf ekf(b.ids_model, cfg, EstimatorMode::kThesis);
    ekf.initialize(b.truth.q, b.truth.v, Vec7::Zero());
    Vec7 d = Vec7::Zero();
    d(4) = 5.0;
    double worst = 0.0;
    double own = 0.0;
    for (int k = 0; k < 130; ++k) {
      const double t0 = 0.01 * k;
      b.advance(t0, k >= 100 ? &d : nullptr);
      ekf.predict(t0, 10, b.track, Vec7::Zero(), false);
      const Vec7 est = ekf.update(b.truth.q, b.truth.v, true).disturbance;
      if (k >= 100) {
        own = std::max(own, std::abs(est(4) - 5.0));
        Vec7 others = est;
        others(4) = 0.0;
        worst = std::max(worst, others.cwiseAbs().maxCoeff());
      }
    }
    return std::make_pair(worst, own);
  };
  const auto first_order = leak(0);
  const auto refined = leak(1);
  EXPECT_LT(refined.first, 0.1 * first_order.first);
  EXPECT_LT(refined.first, 1e-3);
  EXPECT_LT(refined.second, 1e-3);
}
