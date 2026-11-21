#include <gtest/gtest.h>

#include <atomic>
#include <cmath>
#include <cstdlib>
#include <new>

#include "iiwa_controllers/encoder_model.hpp"
#include "iiwa_controllers/tracking_core.hpp"
#include "iiwa_test_model.hpp"

using iiwa_control_law::computeTorque;
using iiwa_control_law::Gains;
using iiwa_control_law::holdTorque;
using iiwa_control_law::referenceAt;
using iiwa_control_law::RobotModel;
using iiwa_control_law::Vec7;
using iiwa_controllers::EncoderConfig;
using iiwa_controllers::EncoderModel;
using iiwa_controllers::MeasurementDecimator;
using iiwa_controllers::TickClock;
using iiwa_controllers::TrackingConfig;
using iiwa_controllers::TrackingCore;
using iiwa_controllers::TrackingState;
using iiwa_controllers::test::hold;
using iiwa_controllers::test::homePose;
using iiwa_controllers::test::iiwaModel;
using iiwa_controllers::test::sinusoid;

namespace {

std::atomic<long> g_allocations{0};
std::atomic<bool> g_counting{false};

void* allocate(std::size_t n, std::size_t alignment) {
  if (g_counting.load(std::memory_order_relaxed)) {
    g_allocations.fetch_add(1, std::memory_order_relaxed);
  }
  const std::size_t size = n == 0 ? 1 : n;
  void* p = alignment <= alignof(std::max_align_t)
                ? std::malloc(size)
                : std::aligned_alloc(alignment, (size + alignment - 1) / alignment * alignment);
  if (p == nullptr) throw std::bad_alloc();
  return p;
}

}  // namespace

void* operator new(std::size_t n) { return allocate(n, alignof(std::max_align_t)); }
void* operator new[](std::size_t n) { return allocate(n, alignof(std::max_align_t)); }
void* operator new(std::size_t n, std::align_val_t a) {
  return allocate(n, static_cast<std::size_t>(a));
}
void* operator new[](std::size_t n, std::align_val_t a) {
  return allocate(n, static_cast<std::size_t>(a));
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }
void operator delete(void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete(void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }

TEST(TrackingCore, HoldsAndReportsHaltedUntilTheFirstReference) {
  RobotModel m = iiwaModel();
  RobotModel g = iiwaModel();
  TrackingCore core(m, TrackingConfig{});
  const Vec7 q = homePose();
  const Vec7 v = Vec7::Constant(0.1);
  const auto& out = core.update(0.0, 0.001, q, v, false);
  EXPECT_EQ(out.state, TrackingState::kWaiting);
  EXPECT_TRUE(out.halted);
  EXPECT_EQ(out.reference_seq, 0u);
  const Vec7 expected = holdTorque(Gains::iiwaDefault(), q, v, q, g.gravity(q));
  EXPECT_LT((out.torque - expected).norm(), 1e-9);
}

TEST(TrackingCore, FirstReferenceIsBlendedInThenTrackedWithTheSharedLaw) {
  RobotModel m = iiwaModel();
  RobotModel g = iiwaModel();
  TrackingConfig c;
  c.resume_time = 0.5;
  TrackingCore core(m, c);
  const Vec7 q = homePose();
  core.update(0.0, 0.001, q, Vec7::Zero(), false);
  Vec7 target = q;
  target(3) += 0.1;
  core.setReference(hold(0.001, 1, target));
  const auto& start = core.update(0.001, 0.001, q, Vec7::Zero(), false);
  EXPECT_EQ(start.state, TrackingState::kResuming);
  EXPECT_TRUE(start.halted);
  EXPECT_EQ(start.reference_seq, 1u);
  EXPECT_LT((start.torque - g.gravity(q)).norm(), 1e-9);
  core.setReference(hold(0.251, 2, target));
  const auto& mid = core.update(0.251, 0.001, q, Vec7::Zero(), false);
  EXPECT_NEAR(mid.torque(3) - g.gravity(q)(3), c.gains.kp(3) * 0.05, 1e-6);
  const auto last = hold(0.502, 3, target);
  core.setReference(last);
  const auto& done = core.update(0.502, 0.001, q, Vec7::Zero(), false);
  EXPECT_EQ(done.state, TrackingState::kTracking);
  EXPECT_FALSE(done.halted);
  const Vec7 expected = computeTorque(c.gains, q, Vec7::Zero(),
                                      referenceAt(last, 0.502, c.reference_hold), g.gravity(q));
  EXPECT_LT((done.torque - expected).norm(), 1e-9);
}

TEST(TrackingCore, PayloadContextSwitchesTheGravityFeedforward) {
  RobotModel m = iiwaModel();
  RobotModel loaded = iiwaModel();
  RobotModel bare = iiwaModel();
  iiwa_control_law::Payload p = iiwa_control_law::defaultPayload();
  p.mass = 3.0;
  loaded.setPayload(p);
  TrackingConfig c;
  c.resume_time = 0.0;
  TrackingCore core(m, c);
  const Vec7 q = homePose();
  auto s = hold(0.0, 1, q);
  s.payload_context = 1;
  s.payload_mass = 3.0;
  core.setReference(s);
  core.update(0.0, 0.001, q, Vec7::Zero(), false);
  const auto& with = core.update(0.001, 0.001, q, Vec7::Zero(), false);
  EXPECT_EQ(with.state, TrackingState::kTracking);
  EXPECT_LT((with.torque - computeTorque(c.gains, q, Vec7::Zero(),
                                         referenceAt(s, 0.001, c.reference_hold),
                                         loaded.gravity(q)))
                .norm(),
            1e-9);
  s.seq = 2;
  s.stamp = 0.002;
  s.payload_context = 0;
  core.setReference(s);
  const auto& without = core.update(0.002, 0.001, q, Vec7::Zero(), false);
  EXPECT_LT((without.torque - computeTorque(c.gains, q, Vec7::Zero(),
                                            referenceAt(s, 0.002, c.reference_hold),
                                            bare.gravity(q)))
                .norm(),
            1e-9);
  EXPECT_GT((loaded.gravity(q) - bare.gravity(q)).norm(), 5.0);
}

TEST(TrackingCore, AlarmHaltsAtTheMeasuredPositionOnlyWhenEnabled) {
  for (const bool enabled : {false, true}) {
    RobotModel m = iiwaModel();
    TrackingConfig c;
    c.resume_time = 0.0;
    c.halt_on_alarm = enabled;
    TrackingCore core(m, c);
    const Vec7 q = homePose();
    for (int k = 0; k < 5; ++k) {
      core.setReference(hold(0.001 * k, static_cast<std::uint64_t>(k + 1), q));
      core.update(0.001 * k, 0.001, q, Vec7::Zero(), false);
    }
    Vec7 moved = q;
    moved(1) += 0.02;
    core.setReference(hold(0.005, 6, q));
    const auto& out = core.update(0.005, 0.001, moved, Vec7::Zero(), true);
    if (enabled) {
      EXPECT_EQ(out.state, TrackingState::kHalted);
      EXPECT_TRUE(out.halted);
      EXPECT_LT((core.holdPosition() - moved).norm(), 1e-15);
    } else {
      EXPECT_EQ(out.state, TrackingState::kTracking);
      EXPECT_FALSE(out.halted);
    }
  }
}

TEST(TrackingCore, ClearedAlarmResumesWithABlendFromTheHoldPosition) {
  RobotModel m = iiwaModel();
  RobotModel g = iiwaModel();
  TrackingConfig c;
  c.resume_time = 0.2;
  c.halt_on_alarm = true;
  TrackingCore core(m, c);
  const Vec7 q = homePose();
  core.setReference(hold(0.0, 1, q));
  core.update(0.0, 0.001, q, Vec7::Zero(), false);
  EXPECT_EQ(core.update(0.3, 0.001, q, Vec7::Zero(), false).state, TrackingState::kTracking);
  EXPECT_EQ(core.update(0.31, 0.001, q, Vec7::Zero(), true).state, TrackingState::kHalted);
  Vec7 target = q;
  target(0) += 0.1;
  core.setReference(hold(0.32, 2, target));
  const auto& r0 = core.update(0.32, 0.001, q, Vec7::Zero(), false);
  EXPECT_EQ(r0.state, TrackingState::kResuming);
  EXPECT_TRUE(r0.halted);
  EXPECT_LT((r0.torque - g.gravity(q)).norm(), 1e-9);
  const auto& r1 = core.update(0.42, 0.001, q, Vec7::Zero(), false);
  EXPECT_NEAR(r1.torque(0) - g.gravity(q)(0), c.gains.kp(0) * 0.05, 1e-6);
  EXPECT_EQ(core.update(0.53, 0.001, q, Vec7::Zero(), false).state, TrackingState::kTracking);
}

TEST(TrackingCore, StaleReferenceHoldsUntilAFreshSampleArrives) {
  RobotModel m = iiwaModel();
  TrackingConfig c;
  c.resume_time = 0.0;
  c.reference_timeout = 0.1;
  TrackingCore core(m, c);
  const Vec7 q = homePose();
  core.setReference(hold(0.0, 1, q));
  EXPECT_EQ(core.update(0.0, 0.001, q, Vec7::Zero(), false).state, TrackingState::kTracking);
  Vec7 moved = q;
  moved(2) += 0.01;
  const auto& starved = core.update(0.2, 0.001, moved, Vec7::Zero(), false);
  EXPECT_EQ(starved.state, TrackingState::kStarved);
  EXPECT_TRUE(starved.halted);
  EXPECT_LT((core.holdPosition() - moved).norm(), 1e-15);
  core.setReference(hold(0.25, 2, q));
  EXPECT_EQ(core.update(0.25, 0.001, moved, Vec7::Zero(), false).state, TrackingState::kTracking);
}

TEST(TrackingCore, TorqueIsClampedToTheEffortLimits) {
  RobotModel m = iiwaModel();
  TrackingConfig c;
  c.resume_time = 0.0;
  TrackingCore core(m, c);
  core.setReference(hold(0.0, 1, homePose() + Vec7::Constant(1.0)));
  const auto& out = core.update(0.0, 0.001, homePose(), Vec7::Zero(), false);
  for (int i = 0; i < 7; ++i) {
    EXPECT_DOUBLE_EQ(out.torque(i), c.gains.effort_limit(i)) << "joint " << i;
  }
}

TEST(TrackingCore, CountsOverrunsAgainstTheNominalPeriod) {
  RobotModel m = iiwaModel();
  TrackingCore core(m, TrackingConfig{});
  const Vec7 q = homePose();
  core.update(0.0, 0.001, q, Vec7::Zero(), false);
  core.update(0.004, 0.004, q, Vec7::Zero(), false);
  core.update(0.005, 0.001, q, Vec7::Zero(), false);
  EXPECT_EQ(core.overruns(), 1);
}

TEST(TrackingCore, TracksASinusoidOnAPinocchioPlantWithFrictionAndEncoderNoise) {
  RobotModel plant = iiwaModel();
  RobotModel controller = iiwaModel();
  TrackingCore core(controller, TrackingConfig{});
  EncoderModel encoder(EncoderConfig{});
  Vec7 friction;
  friction << 4.0, 4.0, 2.0, 2.0, 0.4, 0.2, 0.1;
  Vec7 q = sinusoid(0.0, 0).position;
  Vec7 v = sinusoid(0.0, 0).velocity;
  double worst = 0.0;
  bool halted_after_blend = false;
  for (int k = 0; k < 4000; ++k) {
    const double t = 0.001 * k;
    if (k % 10 == 0) core.setReference(sinusoid(t, static_cast<std::uint64_t>(k / 10)));
    Vec7 qm;
    Vec7 vm;
    encoder.sense(q, v, qm, vm);
    const auto& out = core.update(t, 0.001, qm, vm, false);
    if (t > 0.6 && out.halted) halted_after_blend = true;
    for (int s = 0; s < 4; ++s) {
      const Vec7 a = plant.forwardDynamics(q, v, out.torque - friction.cwiseProduct(v));
      v += 0.00025 * a;
      q += 0.00025 * v;
    }
    if (t > 2.0) worst = std::max(worst, (q - sinusoid(t, 0).position).cwiseAbs().maxCoeff());
  }
  EXPECT_FALSE(halted_after_blend);
  EXPECT_LT(worst, 0.02);
}

TEST(TrackingCore, UpdateNeverAllocates) {
  RobotModel m = iiwaModel();
  TrackingConfig c;
  c.halt_on_alarm = true;
  TrackingCore core(m, c);
  EncoderModel encoder(EncoderConfig{});
  Vec7 q = homePose();
  Vec7 qm;
  Vec7 vm;
  g_allocations = 0;
  for (int k = 0; k < 2000; ++k) {
    const double t = 0.001 * k;
    auto s = sinusoid(t, static_cast<std::uint64_t>(k));
    s.payload_context = k > 1000 ? 1 : 0;
    s.payload_mass = 4.0;
    g_counting = true;
    core.setReference(s);
    encoder.sense(q, Vec7::Zero(), qm, vm);
    core.update(t, 0.001, qm, vm, k > 1500 && k < 1600);
    g_counting = false;
  }
  EXPECT_EQ(g_allocations.load(), 0);
}

TEST(MeasurementDecimator, PublishesEveryNthTickAndReportsHaltsAnywhereInTheWindow) {
  MeasurementDecimator d(10);
  bool window_halted = false;
  EXPECT_TRUE(d.tick(false, window_halted));
  EXPECT_FALSE(window_halted);
  for (int k = 1; k < 10; ++k) EXPECT_FALSE(d.tick(k == 3, window_halted));
  EXPECT_TRUE(d.tick(false, window_halted));
  EXPECT_TRUE(window_halted);
  for (int k = 1; k < 10; ++k) d.tick(false, window_halted);
  EXPECT_TRUE(d.tick(false, window_halted));
  EXPECT_FALSE(window_halted);
}

TEST(MeasurementDecimator, AHaltOnThePublishTickCarriesIntoTheNextWindow) {
  MeasurementDecimator d(10);
  bool window_halted = false;
  d.tick(false, window_halted);
  for (int k = 1; k < 10; ++k) d.tick(false, window_halted);
  EXPECT_TRUE(d.tick(true, window_halted));
  EXPECT_TRUE(window_halted);
  for (int k = 1; k < 10; ++k) d.tick(false, window_halted);
  EXPECT_TRUE(d.tick(false, window_halted));
  EXPECT_TRUE(window_halted);
}

TEST(MeasurementDecimator, ADroppedPublicationDefersItsHaltFlag) {
  MeasurementDecimator d(2);
  bool window_halted = false;
  d.tick(false, window_halted);
  d.tick(true, window_halted);
  EXPECT_TRUE(d.tick(false, window_halted));
  EXPECT_TRUE(window_halted);
  d.defer(window_halted);
  d.tick(false, window_halted);
  EXPECT_TRUE(d.tick(false, window_halted));
  EXPECT_TRUE(window_halted);
}

TEST(TickClock, AdvancesOnePeriodPerUpdateWhateverTheExternalClockDoes) {
  TickClock clock(1000000);
  const std::int64_t t0 = 5003000000;
  EXPECT_EQ(clock.next(t0), t0);
  EXPECT_EQ(clock.next(t0), t0 + 1000000);
  EXPECT_EQ(clock.next(t0 + 9000000), t0 + 2000000);
  for (int i = 0; i < 9996; ++i) {
    clock.next(0);
  }
  EXPECT_EQ(clock.next(1), t0 + 9999LL * 1000000);
  EXPECT_EQ(clock.period_ns(), 1000000);
}

TEST(TickClock, ResetAnchorsOnTheNextExternalTime) {
  TickClock clock(1000000);
  clock.next(100);
  clock.next(200);
  clock.reset();
  EXPECT_EQ(clock.next(7000000000), 7000000000);
  EXPECT_EQ(clock.next(7000000000), 7001000000);
  EXPECT_EQ(TickClock(0).period_ns(), 1);
}