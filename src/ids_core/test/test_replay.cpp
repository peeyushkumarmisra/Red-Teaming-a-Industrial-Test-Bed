// =============================================================================
//  test_replay.cpp - offline replay on synthetic recordings
// -----------------------------------------------------------------------------
//  Records runs from the synthetic cell, then checks: .npy round trip and rejection of
//  other formats, wrong column counts, a calibration from a recording carries the live
//  fingerprint, an armed replay of nominal data raises no alarm, and a 0.02 rad setpoint
//  step on a4 is detected within 0.1 s with a4 as the primary joint.
// =============================================================================

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "ids_core/replay.hpp"
#include "synthetic_cell.hpp"

using ids_core::Array2;
using ids_core::IdsConfig;
using ids_core::IdsPipeline;
using ids_core::ReplayOptions;
using ids_core::ReplayResult;
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

struct Recording {
  Array2 references;
  Array2 measurements;
};

Recording record(double seconds, std::uint64_t seed, const Attack& attack = Attack{}) {
  IdsPipeline sink(IdsConfig{}, urdf());
  CellConfig cc;
  cc.seed = seed;
  SyntheticCell cell(cc);
  cell.setAttack(attack);
  std::vector<iiwa_control_law::ReferenceSample> log;
  cell.logReferences(&log);
  Recording rec;
  const long n = std::lround(seconds / cc.ids_dt);
  for (long k = 0; k < n; ++k) {
    ids_core::appendMeasurement(rec.measurements, cell.step(sink));
  }
  for (const auto& s : log) {
    ids_core::appendReference(rec.references, s);
  }
  return rec;
}

const Recording& nominal() {
  static const Recording r = record(70.0, 100);
  return r;
}

const ReplayResult& calibrated() {
  static const ReplayResult r = [] {
    ReplayOptions o;
    o.warmup = 0.0;
    o.calibration_duration = 60.0;
    return ids_core::replay(IdsConfig{}, urdf(), nominal().references, nominal().measurements, o);
  }();
  return r;
}

std::vector<std::size_t> onsets(const Array2& w) {
  std::vector<std::size_t> rows;
  for (std::size_t i = 0; i < w.rows; ++i) {
    if (w.at(i, 6) > 0.5) {
      rows.push_back(i);
    }
  }
  return rows;
}

}  // namespace

TEST(Npy, RoundTripKeepsShapeAndValues) {
  Array2 a;
  a.rows = 3;
  a.cols = 4;
  for (int i = 0; i < 12; ++i) {
    a.data.push_back(i * 0.5 - 2.0);
  }
  const std::string path = ::testing::TempDir() + "/round_trip.npy";
  ids_core::writeNpy(path, a);
  const Array2 b = ids_core::readNpy(path);
  EXPECT_EQ(b.rows, 3u);
  EXPECT_EQ(b.cols, 4u);
  EXPECT_EQ(b.data, a.data);
  std::remove(path.c_str());
}

TEST(Npy, RejectsFilesThatAreNotFloat64Matrices) {
  const std::string path = ::testing::TempDir() + "/not_npy.npy";
  std::ofstream(path) << "hello";
  EXPECT_THROW(ids_core::readNpy(path), std::runtime_error);
  EXPECT_THROW(ids_core::readNpy(::testing::TempDir() + "/missing.npy"), std::runtime_error);
  std::remove(path.c_str());
}

TEST(Replay, RejectsArraysWithTheWrongColumns) {
  Array2 bad;
  bad.cols = 3;
  ReplayOptions o;
  EXPECT_THROW(ids_core::replay(IdsConfig{}, urdf(), bad, nominal().measurements, o),
               std::runtime_error);
  EXPECT_THROW(ids_core::replay(IdsConfig{}, urdf(), nominal().references, bad, o),
               std::runtime_error);
}

TEST(Replay, CalibratesFromARecordingWithTheLiveFingerprint) {
  const ReplayResult& r = calibrated();
  ASSERT_TRUE(r.calibrated);
  EXPECT_EQ(r.calibration.fingerprint, IdsPipeline(IdsConfig{}, urdf()).fingerprint());
  EXPECT_GT(r.calibration.samples, 4000);
  EXPECT_GT(r.calibration.limit, 3.0);
  ReplayOptions too_long;
  too_long.warmup = 0.0;
  too_long.calibration_duration = 500.0;
  EXPECT_THROW(ids_core::replay(IdsConfig{}, urdf(), nominal().references,
                                nominal().measurements, too_long),
               std::runtime_error);
}

TEST(Replay, ArmedReplayOfNominalDataRaisesNoAlarm) {
  ReplayOptions o;
  o.warmup = 0.0;
  o.calibration = &calibrated().calibration;
  const ReplayResult r =
      ids_core::replay(IdsConfig{}, urdf(), nominal().references, nominal().measurements, o);
  EXPECT_EQ(r.windows.rows, nominal().measurements.rows);
  EXPECT_EQ(r.windows.cols, ids_core::kReplayColumns);
  EXPECT_EQ(r.onsets, 0);
  EXPECT_EQ(r.windows.at(r.windows.rows - 1, 1), 2.0);
}

TEST(Replay, ArmedReplayDetectsASetpointStepOnTheAttackedJoint) {
  Attack a;
  a.kind = AttackKind::kSetpointStep;
  a.joint = 3;
  a.magnitude = 0.02;
  a.start = 30.0;
  const Recording rec = record(40.0, 7, a);
  ReplayOptions o;
  o.warmup = 0.0;
  o.calibration = &calibrated().calibration;
  const ReplayResult r =
      ids_core::replay(IdsConfig{}, urdf(), rec.references, rec.measurements, o);
  const auto rows = onsets(r.windows);
  ASSERT_FALSE(rows.empty());
  const double t = r.windows.at(rows.front(), 0);
  EXPECT_GE(t, 30.0);
  EXPECT_LT(t, 30.1);
  EXPECT_EQ(r.windows.at(rows.front(), 5), 3.0);
  EXPECT_GT(r.windows.at(rows.front(), 15 + 3), calibrated().calibration.limit);
  const ReplayResult again =
      ids_core::replay(IdsConfig{}, urdf(), rec.references, rec.measurements, o);
  EXPECT_EQ(again.windows.data, r.windows.data);
}
