#include <gtest/gtest.h>

#include <fstream>
#include <random>
#include <sstream>
#include <stdexcept>

#include "ids_core/ids_pipeline.hpp"
#include "test_helpers.hpp"

using ids_core::Calibration;
using ids_core::ClosedLoopModel;
using ids_core::IdsConfig;
using ids_core::IdsPipeline;
using ids_core::InputSource;
using ids_core::Measurement;
using ids_core::Mode;
using ids_core::ModelConfig;
using ids_core::ReferenceTrack;
using ids_core::State;
using iiwa_control_law::RobotModel;
using iiwa_control_law::Vec7;

namespace {

std::string urdf() {
  std::ifstream f(ids_core::test::urdfPath());
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

struct Cell {
  RobotModel robot = RobotModel::fromUrdfFile(ids_core::test::urdfPath());
  ModelConfig config;
  ReferenceTrack track;
  ReferenceTrack hold_track;
  ClosedLoopModel model{robot, config, InputSource::kTrustedReference};
  State x;
  std::uint64_t seq = 0;
  std::mt19937 rng{11};
  double t = 0.0;
  bool holding = false;

  Cell() {
    const auto s = ids_core::test::sinusoidSample(0.0, 0);
    x.q = s.position;
    x.v = s.velocity;
  }

  void hold() {
    iiwa_control_law::ReferenceSample h;
    h.stamp = t;
    h.position = x.q;
    hold_track.clear();
    hold_track.push(h);
    holding = true;
  }

  Measurement step(IdsPipeline& ids, const Vec7* command_bias = nullptr, bool report = true) {
    const auto s = ids_core::test::sinusoidSample(t, seq++);
    track.push(s);
    ids.addReference(s);
    model.propagate(x, t, 10, holding ? hold_track : track, Vec7::Zero(), nullptr, nullptr);
    t += 0.01;
    std::normal_distribution<double> n(0.0, 1e-3);
    Measurement m;
    m.stamp = t;
    m.q = x.q;
    m.v = x.v;
    for (int i = 0; i < 7; ++i) m.v(i) += n(rng);
    m.reference_seq = s.seq;
    m.tau = model.nominalTorque(m.q, m.v, s, t);
    if (command_bias != nullptr) m.tau += *command_bias;
    m.halted = holding && report;
    return m;
  }
};

Calibration calibrate(IdsPipeline& ids, Cell& cell, int samples) {
  ids.startCalibration();
  for (int k = 0; k < samples; ++k) ids.process(cell.step(ids));
  return ids.finishCalibration();
}

}  // namespace

TEST(Pipeline, WaitsUntilCalibratedOrArmed) {
  IdsPipeline ids(IdsConfig{}, urdf());
  Cell cell;
  EXPECT_EQ(ids.mode(), Mode::kWaiting);
  EXPECT_FALSE(ids.process(cell.step(ids)).valid);
}

TEST(Pipeline, CalibrateOnceThenArmAFreshPipelineFromTheFile) {
  IdsConfig cfg;
  const std::string u = urdf();
  Calibration cal;
  {
    IdsPipeline ids(cfg, u);
    Cell cell;
    cal = calibrate(ids, cell, 1500);
    EXPECT_EQ(cal.fingerprint, ids.fingerprint());
    EXPECT_GE(cal.samples, 1400);
  }
  const std::string path = "/tmp/ids_pipeline_calibration.yaml";
  ids_core::saveCalibration(cal, path);
  IdsPipeline fresh(cfg, u);
  fresh.arm(ids_core::loadCalibration(path));
  EXPECT_EQ(fresh.mode(), Mode::kArmed);
  Cell cell;
  int alarms = 0;
  for (int k = 0; k < 1500; ++k) {
    const auto& out = fresh.process(cell.step(fresh));
    if (out.valid && out.alarm) ++alarms;
  }
  EXPECT_EQ(alarms, 0);
}

TEST(Pipeline, RefusesCalibrationFromADifferentConfiguration) {
  IdsConfig cfg;
  IdsPipeline ids(cfg, urdf());
  Cell cell;
  Calibration cal = calibrate(ids, cell, 300);
  IdsConfig other = cfg;
  other.model.gains.kp(0) = 700.0;
  IdsPipeline changed(other, urdf());
  EXPECT_THROW(changed.arm(cal), std::runtime_error);
  EXPECT_EQ(changed.mode(), Mode::kFault);
}

TEST(Pipeline, CommandCheckFlagsTamperedTorqueOnTheRightJoint) {
  IdsConfig cfg;
  IdsPipeline ids(cfg, urdf());
  Cell cell;
  ids.arm(calibrate(ids, cell, 600));
  for (int k = 0; k < ids.detectorWarmupWindows(); ++k) ids.process(cell.step(ids));
  Vec7 bias = Vec7::Zero();
  bias(2) = cfg.model.gains.kp(2) * 0.02;
  int first = -1;
  for (int k = 0; k < 20; ++k) {
    const auto& out = ids.process(cell.step(ids, &bias));
    if (out.alarm && first < 0) {
      first = k;
      EXPECT_TRUE(out.command_alarm(2));
      EXPECT_FALSE(out.command_alarm(0));
    }
  }
  EXPECT_GE(first, 0);
  EXPECT_LE(first, 6);
  ids.resetAlarm();
  EXPECT_FALSE(ids.process(cell.step(ids)).latched);
}

TEST(Pipeline, HaltedWindowsAreSkippedAndTrackingResumesWithoutAlarm) {
  IdsConfig cfg;
  const std::string u = urdf();
  Calibration cal;
  {
    IdsPipeline ids(cfg, u);
    Cell cell;
    cal = calibrate(ids, cell, 600);
  }
  const auto run = [&](bool report_halt) {
    IdsPipeline ids(cfg, u);
    ids.arm(cal);
    Cell cell;
    int alarms = 0;
    int valid_while_holding = 0;
    const int w = ids.detectorWarmupWindows();
    for (int k = 0; k < w + 400; ++k) {
      if (k == w + 150) cell.hold();
      if (k == w + 200) cell.holding = false;
      const auto& out = ids.process(cell.step(ids, nullptr, report_halt));
      if (out.valid && out.alarm) ++alarms;
      if (cell.holding && out.valid) ++valid_while_holding;
    }
    return std::make_pair(alarms, valid_while_holding);
  };
  const auto reported = run(true);
  EXPECT_EQ(reported.first, 0);
  EXPECT_EQ(reported.second, 0);
  const auto hidden = run(false);
  EXPECT_GT(hidden.first, 0);
}

TEST(Pipeline, DetectorWaitsForTheEwmaWarmUpAfterArmingAndReset) {
  IdsConfig cfg;
  IdsPipeline ids(cfg, urdf());
  Cell cell;
  ids.arm(calibrate(ids, cell, 600));
  EXPECT_EQ(ids.detectorWarmupWindows(), 399);
  Vec7 bias = Vec7::Zero();
  bias(3) = cfg.model.gains.kp(3) * 0.02;
  for (int k = 0; k < ids.detectorWarmupWindows(); ++k) {
    EXPECT_FALSE(ids.process(cell.step(ids, &bias)).alarm) << k;
  }
  bool alarmed = false;
  for (int k = 0; k < 10; ++k) alarmed = ids.process(cell.step(ids, &bias)).alarm || alarmed;
  EXPECT_TRUE(alarmed);
  ids.resetAlarm();
  for (int k = 0; k < ids.detectorWarmupWindows(); ++k) {
    EXPECT_FALSE(ids.process(cell.step(ids, &bias)).alarm) << k;
  }
}

TEST(Pipeline, EachNewOnsetNamesItsOwnPrimaryJoint) {
  IdsConfig cfg;
  cfg.detector.latch = false;
  IdsPipeline ids(cfg, urdf());
  Cell cell;
  ids.arm(calibrate(ids, cell, 600));
  for (int k = 0; k < ids.detectorWarmupWindows(); ++k) ids.process(cell.step(ids));
  const auto onset_on = [&](int joint) {
    Vec7 bias = Vec7::Zero();
    bias(joint) = cfg.model.gains.kp(joint) * 0.02;
    for (int k = 0; k < 20; ++k) {
      const auto& out = ids.process(cell.step(ids, &bias));
      if (out.alarm) return out.primary_joint;
    }
    return -1;
  };
  EXPECT_EQ(onset_on(2), 2);
  bool cleared = false;
  for (int k = 0; k < 300 && !cleared; ++k) cleared = !ids.process(cell.step(ids)).alarm;
  ASSERT_TRUE(cleared);
  EXPECT_EQ(onset_on(4), 4);
}