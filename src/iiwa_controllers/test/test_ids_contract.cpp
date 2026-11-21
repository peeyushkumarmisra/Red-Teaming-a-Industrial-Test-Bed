#include <gtest/gtest.h>

#include <cstdint>

#include "ids_core/ids_pipeline.hpp"
#include "iiwa_controllers/encoder_model.hpp"
#include "iiwa_controllers/tracking_core.hpp"
#include "iiwa_test_model.hpp"

using iiwa_control_law::Vec7;
using iiwa_controllers::EncoderConfig;
using iiwa_controllers::EncoderModel;
using iiwa_controllers::MeasurementDecimator;
using iiwa_controllers::TrackingConfig;
using iiwa_controllers::TrackingCore;

namespace {

class Cell {
 public:
  Cell(const TrackingConfig& config, std::uint64_t seed)
      : plant_(iiwa_controllers::test::iiwaModel()),
        controller_(iiwa_controllers::test::iiwaModel()),
        core_(controller_, config),
        encoder_(EncoderConfig{20, 1e-3, seed}) {
    q_ = iiwa_controllers::test::sinusoid(0.0, 0).position;
    v_ = iiwa_controllers::test::sinusoid(0.0, 0).velocity;
    friction_ << 4.0, 4.0, 2.0, 2.0, 0.4, 0.2, 0.1;
  }

  void attack(int joint, double offset) {
    attack_joint_ = joint;
    attack_offset_ = offset;
  }

  void alarm(bool on) { alarm_ = on; }

  ids_core::Measurement step(ids_core::IdsPipeline& ids) {
    ids_core::Measurement m;
    for (int c = 0; c < 10; ++c) {
      const double t = 0.001 * static_cast<double>(tick_);
      if (tick_ % 10 == 0) {
        auto s = iiwa_controllers::test::sinusoid(t, seq_++);
        ids.addReference(s);
        if (attack_joint_ >= 0) s.position(attack_joint_) += attack_offset_;
        core_.setReference(s);
      }
      Vec7 qm;
      Vec7 vm;
      encoder_.sense(q_, v_, qm, vm);
      const auto& out = core_.update(t, 0.001, qm, vm, alarm_);
      bool window_halted = out.halted;
      if (decimator_.tick(out.halted, window_halted)) {
        m.stamp = t;
        m.q = qm;
        m.v = vm;
        m.tau = out.torque;
        m.reference_seq = out.reference_seq;
        m.halted = window_halted;
      }
      for (int s = 0; s < 4; ++s) {
        const Vec7 a = plant_.forwardDynamics(q_, v_, out.torque - friction_.cwiseProduct(v_));
        v_ += 0.00025 * a;
        q_ += 0.00025 * v_;
      }
      ++tick_;
    }
    return m;
  }

 private:
  iiwa_control_law::RobotModel plant_;
  iiwa_control_law::RobotModel controller_;
  TrackingCore core_;
  EncoderModel encoder_;
  MeasurementDecimator decimator_{10};
  Vec7 q_;
  Vec7 v_;
  Vec7 friction_;
  long tick_ = 0;
  std::uint64_t seq_ = 0;
  int attack_joint_ = -1;
  double attack_offset_ = 0.0;
  bool alarm_ = false;
};

const ids_core::Calibration& calibration() {
  static const ids_core::Calibration cal = [] {
    ids_core::IdsPipeline ids(ids_core::IdsConfig{}, iiwa_controllers::test::iiwaUrdf());
    Cell cell(TrackingConfig{}, 3);
    ids.startCalibration();
    for (int k = 0; k < 4200; ++k) {
      const auto m = cell.step(ids);
      if (k >= 200) ids.process(m);
    }
    return ids.finishCalibration();
  }();
  return cal;
}

}  // namespace

TEST(ControllerIdsContract, NominalTrackingWithConstantFrictionRaisesNoAlarm) {
  ids_core::IdsPipeline ids(ids_core::IdsConfig{}, iiwa_controllers::test::iiwaUrdf());
  ids.arm(calibration());
  Cell cell(TrackingConfig{}, 9);
  int valid = 0;
  int alarms = 0;
  for (int k = 0; k < 6000; ++k) {
    const auto m = cell.step(ids);
    if (k < 200) continue;
    const auto& out = ids.process(m);
    if (out.valid) ++valid;
    if (out.valid && out.alarm) ++alarms;
  }
  EXPECT_GT(valid, 5700);
  EXPECT_EQ(alarms, 0);
}

TEST(ControllerIdsContract, SetpointMitmIsDetectedHaltedAndResumedWithoutReAlarm) {
  for (int j = 0; j < 7; ++j) {
    TrackingConfig tc;
    tc.halt_on_alarm = true;
    tc.resume_time = 0.3;
    ids_core::IdsPipeline ids(ids_core::IdsConfig{}, iiwa_controllers::test::iiwaUrdf());
    ids.arm(calibration());
    Cell cell(tc, 20 + static_cast<std::uint64_t>(j));
    double detected = -1.0;
    int primary = -1;
    bool halted = false;
    bool resumed = false;
    int alarms_after_resume = 0;
    bool controller_alarm = false;
    for (int k = 0; k < 1500; ++k) {
      if (k == 600) cell.attack(j, 0.02);
      if (k == 800) cell.attack(-1, 0.0);
      if (k == 900) {
        ids.resetAlarm();
        controller_alarm = false;
      }
      cell.alarm(controller_alarm);
      const auto m = cell.step(ids);
      if (k < 200) continue;
      const auto& out = ids.process(m);
      if (out.valid) controller_alarm = out.alarm || out.latched;
      if (out.valid && out.alarm && detected < 0.0) {
        detected = m.stamp - 6.0;
        primary = out.primary_joint;
      }
      if (detected >= 0.0 && k < 900 && m.halted) halted = true;
      if (k > 900 && !m.halted) resumed = true;
      if (resumed && out.valid && out.alarm) ++alarms_after_resume;
    }
    EXPECT_GE(detected, 0.0) << "joint " << j;
    EXPECT_LE(detected, 0.1) << "joint " << j;
    EXPECT_EQ(primary, j);
    EXPECT_TRUE(halted) << "joint " << j;
    EXPECT_TRUE(resumed) << "joint " << j;
    EXPECT_EQ(alarms_after_resume, 0) << "joint " << j;
  }
}