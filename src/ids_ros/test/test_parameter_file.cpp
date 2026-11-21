#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <string>

#include "ids_core/calibration.hpp"
#include "ids_ros/parameter_file.hpp"
#include "ids_ros/parameters.hpp"

using ids_core::EstimatorMode;
using ids_core::IdsConfig;
using ids_core::InputSource;
using ids_ros::configFromParameters;
using ids_ros::parametersFromFile;
using ids_ros::parseOverride;

namespace {

std::string config(const char* name) { return std::string(IDS_ROS_CONFIG_DIR) + "/" + name; }

std::string writeTemp(const char* name, const char* text) {
  const std::string path = ::testing::TempDir() + "/" + name;
  std::ofstream(path) << text;
  return path;
}

}  // namespace

TEST(ParameterFile, V2FileGivesTheDefaultConfiguration) {
  const auto params = parametersFromFile(config("ids.yaml"));
  EXPECT_FALSE(params.count("run_mode"));
  EXPECT_FALSE(params.count("use_sim_time"));
  EXPECT_TRUE(params.count("detector.debounce"));
  const IdsConfig c = configFromParameters(params);
  EXPECT_EQ(c.input_source, InputSource::kTrustedReference);
  EXPECT_EQ(c.mode, EstimatorMode::kExtended);
  EXPECT_EQ(ids_core::configSignature(c), ids_core::configSignature(IdsConfig{}));
}

TEST(ParameterFile, ThesisFileSelectsTheDissertationDesign) {
  const IdsConfig c = configFromParameters(parametersFromFile(config("ids_thesis.yaml")));
  EXPECT_EQ(c.input_source, InputSource::kCommandedTorque);
  EXPECT_EQ(c.mode, EstimatorMode::kThesis);
}

TEST(ParameterFile, OverridesUseTheParameterTypes) {
  auto params = parametersFromFile(config("ids.yaml"));
  for (const char* s : {"detector.debounce=7", "ekf.q_friction=1e-7", "detector.latch=false",
                        "input_source=commanded_torque", "model.kp=1,2,3,4,5,6,7"}) {
    const auto [name, value] = parseOverride(s);
    params[name] = value;
  }
  const IdsConfig c = configFromParameters(params);
  EXPECT_EQ(c.detector.debounce, 7);
  EXPECT_DOUBLE_EQ(c.ekf.q_friction, 1e-7);
  EXPECT_FALSE(c.detector.latch);
  EXPECT_EQ(c.input_source, InputSource::kCommandedTorque);
  EXPECT_DOUBLE_EQ(c.model.gains.kp(6), 7.0);
}

TEST(ParameterFile, BadInputIsRejected) {
  EXPECT_THROW(parseOverride("detector.debounce"), std::invalid_argument);
  EXPECT_THROW(parseOverride("detector.nonsense=1"), std::invalid_argument);
  EXPECT_THROW(parseOverride("detector.debounce=five"), std::invalid_argument);
  EXPECT_THROW(parametersFromFile(writeTemp("no_params.yaml", "a: 1\n")), std::invalid_argument);
  EXPECT_THROW(parametersFromFile(::testing::TempDir() + "/missing.yaml"), std::invalid_argument);
  EXPECT_THROW(parametersFromFile(writeTemp(
                   "bad_type.yaml", "/**:\n  ros__parameters:\n    detector:\n      debounce: x\n")),
               std::invalid_argument);
}

TEST(ParameterFile, NodeSectionsAndPlainSectionsAreBothAccepted) {
  const auto a = parametersFromFile(writeTemp(
      "node.yaml", "ids:\n  ros__parameters:\n    ewma:\n      tau_slow: 3.0\n"));
  const auto b = parametersFromFile(writeTemp(
      "plain.yaml", "ros__parameters:\n  ewma:\n    tau_slow: 3.0\n"));
  EXPECT_DOUBLE_EQ(configFromParameters(a).ewma.tau_slow, 3.0);
  EXPECT_DOUBLE_EQ(configFromParameters(b).ewma.tau_slow, 3.0);
}