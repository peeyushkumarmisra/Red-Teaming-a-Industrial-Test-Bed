#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <random>
#include <sstream>

#include "ids_core/calibration.hpp"
#include "ids_core/sha256.hpp"
#include "test_helpers.hpp"

using ids_core::Calibration;
using ids_core::CalibrationRecorder;
using ids_core::IdsConfig;
using ids_core::Vec7;

namespace {

std::string urdf() {
  std::ifstream f(ids_core::test::urdfPath());
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

std::string replaceOnce(std::string text, const std::string& from, const std::string& to) {
  const std::size_t at = text.find(from);
  EXPECT_NE(at, std::string::npos) << from;
  if (at != std::string::npos) text.replace(at, from.size(), to);
  return text;
}

}  // namespace

TEST(Sha256, KnownVectors) {
  EXPECT_EQ(ids_core::sha256Hex(""),
            "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
  EXPECT_EQ(ids_core::sha256Hex("abc"),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  EXPECT_EQ(ids_core::sha256Hex(std::string(1000, 'a')).size(), 64u);
}

TEST(Fingerprint, ChangesWithModelGainsAndDetectorSettings) {
  const std::string u = urdf();
  IdsConfig c;
  const std::string base = ids_core::computeFingerprint(u, c);
  EXPECT_EQ(base, ids_core::computeFingerprint(u, c));
  IdsConfig gains = c;
  gains.model.gains.kp(3) += 1.0;
  EXPECT_NE(base, ids_core::computeFingerprint(u, gains));
  IdsConfig det = c;
  det.detector.debounce = 3;
  EXPECT_NE(base, ids_core::computeFingerprint(u, det));
  EXPECT_NE(base, ids_core::computeFingerprint(
                      replaceOnce(u, "<mass value=\"2.7\" />", "<mass value=\"2.8\" />"), c));
  EXPECT_NE(base, ids_core::computeFingerprint(
                      replaceOnce(u, "<axis xyz=\"0 -1 0\" />", "<axis xyz=\"0 1 0\" />"), c));
}

TEST(Fingerprint, IgnoresFormattingAndSimulatorOnlyElements) {
  const std::string u = urdf();
  const IdsConfig c;
  const std::string base = ids_core::computeFingerprint(u, c);
  EXPECT_EQ(base, ids_core::computeFingerprint(u + "  \n", c));
  std::string sim = u;
  sim.insert(sim.rfind("</robot>"),
             "<ros2_control name=\"s\" type=\"system\"><hardware><plugin>x</plugin></hardware>"
             "</ros2_control><gazebo><plugin filename=\"f\" name=\"n\"><delay>30</delay>"
             "</plugin></gazebo>");
  EXPECT_EQ(base, ids_core::computeFingerprint(sim, c));
}

TEST(Calibration, SaveAndLoadRoundTrip) {
  Calibration cal;
  cal.fingerprint = "abc123";
  cal.duration_s = 60.0;
  cal.samples = 6000;
  cal.limit = 4.37;
  cal.slow_limit = 4.9;
  cal.physics_mean << 1, 2, 3, 4, 5, 6, 7;
  cal.physics_std = Vec7::Constant(0.25);
  cal.friction_initial = Vec7::Constant(0.5);
  const std::string path = "/tmp/ids_core_calibration_test.yaml";
  ids_core::saveCalibration(cal, path);
  const Calibration back = ids_core::loadCalibration(path);
  EXPECT_EQ(back.fingerprint, cal.fingerprint);
  EXPECT_EQ(back.samples, cal.samples);
  EXPECT_DOUBLE_EQ(back.limit, cal.limit);
  EXPECT_LT((back.physics_mean - cal.physics_mean).norm(), 1e-12);
  EXPECT_LT((back.friction_initial - cal.friction_initial).norm(), 1e-12);
  std::remove(path.c_str());
}

TEST(Calibration, RecorderEstimatesMomentsAndPicksLimits) {
  IdsConfig c;
  CalibrationRecorder rec;
  std::mt19937 rng(5);
  std::normal_distribution<double> n(0.0, 1.0);
  for (int k = 0; k < 6000; ++k) {
    Vec7 p, cmd;
    for (int i = 0; i < 7; ++i) {
      p(i) = 2.0 + 0.5 * n(rng);
      cmd(i) = 1e-6 * n(rng);
    }
    rec.add(p, &cmd);
  }
  const Calibration cal = rec.finalize(c, "fp", Vec7::Zero(), 60.0);
  EXPECT_NEAR(cal.physics_mean(0), 2.0, 0.05);
  EXPECT_NEAR(cal.physics_std(3), 0.5, 0.05);
  EXPECT_DOUBLE_EQ(cal.command_std(0), c.detector.sigma_floor);
  EXPECT_NEAR(cal.physics_scale(0), 1.0, 0.15);
  EXPECT_GT(cal.limit, 3.5);
  EXPECT_LT(cal.limit, 8.0);
  EXPECT_GT(cal.slow_limit, 3.5);
}