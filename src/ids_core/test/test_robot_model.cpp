#include <gtest/gtest.h>

#include <Eigen/Eigenvalues>

#include "ids_core/config.hpp"
#include "iiwa_control_law/robot_model.hpp"
#include "test_helpers.hpp"

using iiwa_control_law::Mat7;
using iiwa_control_law::RobotModel;
using iiwa_control_law::Vec7;

namespace {

RobotModel load() { return RobotModel::fromUrdfFile(ids_core::test::urdfPath()); }

}  // namespace

TEST(RobotModel, LoadsSevenJointsWithLimits) {
  RobotModel m = load();
  EXPECT_EQ(m.jointNames()[0], "joint_a1");
  EXPECT_EQ(m.jointNames()[6], "joint_a7");
  EXPECT_DOUBLE_EQ(m.effortLimits()(0), 320.0);
  EXPECT_DOUBLE_EQ(m.effortLimits()(6), 40.0);
}

TEST(RobotModel, MassMatrixSymmetricPositiveDefinite) {
  RobotModel m = load();
  const Mat7 mm = m.massMatrix(ids_core::test::homePose());
  EXPECT_LT((mm - mm.transpose()).norm(), 1e-12);
  Eigen::SelfAdjointEigenSolver<Mat7> es(mm);
  EXPECT_GT(es.eigenvalues().minCoeff(), 0.0);
}

TEST(RobotModel, InverseMassFromDerivativesMatchesInverse) {
  RobotModel m = load();
  const Vec7 q = ids_core::test::homePose();
  const Vec7 v = Vec7::Constant(0.2);
  Mat7 dq, dv, minv;
  m.forwardDynamicsDerivatives(q, v, m.gravity(q), dq, dv, minv);
  EXPECT_LT((minv * m.massMatrix(q) - Mat7::Identity()).norm(), 1e-8);
}

TEST(RobotModel, GravityEqualsInverseDynamicsAtRest) {
  RobotModel m = load();
  const Vec7 q = ids_core::test::homePose();
  EXPECT_LT((m.gravity(q) - m.inverseDynamics(q, Vec7::Zero(), Vec7::Zero())).norm(), 1e-10);
}

TEST(RobotModel, ForwardAndInverseDynamicsAreConsistent) {
  RobotModel m = load();
  const Vec7 q = ids_core::test::homePose();
  const Vec7 v = Vec7::Constant(-0.3);
  Vec7 a;
  a << 0.5, -0.2, 0.1, 0.3, -0.4, 0.2, 0.1;
  const Vec7 tau = m.inverseDynamics(q, v, a);
  EXPECT_LT((m.forwardDynamics(q, v, tau) - a).norm(), 1e-8);
}

TEST(RobotModel, GravityDerivativeMatchesFiniteDifference) {
  RobotModel m = load();
  const Vec7 q = ids_core::test::homePose();
  Mat7 analytic;
  m.gravityDerivative(q, analytic);
  Mat7 numeric;
  const double eps = 1e-6;
  for (int j = 0; j < 7; ++j) {
    Vec7 qp = q, qm = q;
    qp(j) += eps;
    qm(j) -= eps;
    numeric.col(j) = (m.gravity(qp) - m.gravity(qm)) / (2 * eps);
  }
  EXPECT_LT((analytic - numeric).norm(), 1e-5);
}

TEST(RobotModel, ForwardDynamicsDerivativesMatchFiniteDifference) {
  RobotModel m = load();
  const Vec7 q = ids_core::test::homePose();
  const Vec7 v = Vec7::Constant(0.4);
  const Vec7 tau = m.gravity(q) + Vec7::Constant(2.0);
  Mat7 dq, dv, minv;
  m.forwardDynamicsDerivatives(q, v, tau, dq, dv, minv);
  const double eps = 1e-6;
  Mat7 nq, nv;
  for (int j = 0; j < 7; ++j) {
    Vec7 p = q, n = q;
    p(j) += eps;
    n(j) -= eps;
    nq.col(j) = (m.forwardDynamics(p, v, tau) - m.forwardDynamics(n, v, tau)) / (2 * eps);
    Vec7 vp = v, vn = v;
    vp(j) += eps;
    vn(j) -= eps;
    nv.col(j) = (m.forwardDynamics(q, vp, tau) - m.forwardDynamics(q, vn, tau)) / (2 * eps);
  }
  EXPECT_LT((dq - nq).norm() / nq.norm(), 1e-5);
  EXPECT_LT((dv - nv).norm() / (nv.norm() + 1e-9), 1e-5);
}

TEST(RobotModel, PayloadIsAddedAndRemovedExactly) {
  RobotModel m = load();
  Vec7 q = Vec7::Zero();
  q(1) = 1.5707963;
  const Vec7 g0 = m.gravity(q);
  m.setPayload(ids_core::ModelConfig::defaultPayload());
  const Vec7 g1 = m.gravity(q);
  EXPECT_GT(std::abs(g1(1) - g0(1)), 20.0);
  m.setPayload(iiwa_control_law::Payload{});
  EXPECT_LT((m.gravity(q) - g0).norm(), 1e-12);
}

TEST(RobotModel, CopiesAreIndependent) {
  RobotModel a = load();
  RobotModel b = a;
  b.setPayload(ids_core::ModelConfig::defaultPayload());
  Vec7 q = Vec7::Zero();
  q(1) = 1.0;
  EXPECT_GT((a.gravity(q) - b.gravity(q)).norm(), 1.0);
}