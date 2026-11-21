// =============================================================================
//  test_encoder_model.cpp - the sensor model
// -----------------------------------------------------------------------------
//  Resolution, noise level, seeding, and that the encoder stays inside the IDS's noise model.
// =============================================================================

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

#include "iiwa_controllers/encoder_model.hpp"

using iiwa_control_law::Vec7;
using iiwa_controllers::EncoderConfig;
using iiwa_controllers::EncoderModel;

// Positions land on the encoder grid.
TEST(EncoderModel, QuantisesPositionToTheConfiguredResolution) {
  EncoderConfig c;
  c.position_bits = 16;
  c.velocity_noise = 0.0;
  EncoderModel e(c);
  const double res = 2.0 * std::numbers::pi / 65536.0;
  EXPECT_DOUBLE_EQ(e.resolution(), res);
  const Vec7 q = Vec7::LinSpaced(-2.0, 2.0) + Vec7::Constant(0.0123456789);
  const Vec7 v = Vec7::Constant(0.3);
  Vec7 qm;
  Vec7 vm;
  e.sense(q, v, qm, vm);
  for (int i = 0; i < 7; ++i) {
    EXPECT_LE(std::abs(qm(i) - q(i)), 0.5 * res + 1e-15);
    EXPECT_NEAR(qm(i) / res, std::round(qm(i) / res), 1e-9);
  }
  EXPECT_EQ(vm, v);
}

// Noise has the right size; the same seed gives the same noise.
TEST(EncoderModel, VelocityNoiseHasTheConfiguredSigmaAndIsSeeded) {
  EncoderConfig c;
  c.velocity_noise = 1e-3;
  c.seed = 7;
  EncoderModel a(c);
  EncoderModel b(c);
  const Vec7 q = Vec7::Zero();
  const Vec7 v = Vec7::Constant(0.5);
  double sum = 0.0;
  double sum2 = 0.0;
  const int n = 20000;
  for (int k = 0; k < n; ++k) {
    Vec7 qa, va, qb, vb;
    a.sense(q, v, qa, va);
    b.sense(q, v, qb, vb);
    ASSERT_EQ(va, vb);
    const Vec7 e = va - v;
    sum += e.sum();
    sum2 += e.squaredNorm();
  }
  const double count = 7.0 * n;
  const double mean = sum / count;
  const double sigma = std::sqrt(sum2 / count - mean * mean);
  EXPECT_NEAR(mean, 0.0, 3e-3 / std::sqrt(count) * 3.0);
  EXPECT_NEAR(sigma, 1e-3, 3e-5);
}

// 0 bits disables quantisation.
TEST(EncoderModel, ZeroBitsPassesPositionThrough) {
  EncoderConfig c;
  c.position_bits = 0;
  c.velocity_noise = 0.0;
  EncoderModel e(c);
  const Vec7 q = Vec7::Constant(0.123456789);
  Vec7 qm;
  Vec7 vm;
  e.sense(q, Vec7::Zero(), qm, vm);
  EXPECT_EQ(qm, q);
}

// The IDS's position noise covers the encoder's rounding.
TEST(EncoderModel, DefaultResolutionIsWithinTheIdsPositionNoiseModel) {
  EncoderModel e(EncoderConfig{});
  EXPECT_LT(e.resolution() / std::sqrt(12.0), 1e-5);
}
