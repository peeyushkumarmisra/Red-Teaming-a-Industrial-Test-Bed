#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <vector>

#include "ids_core/calibration.hpp"
#include "ids_ros/parameters.hpp"

using ids_core::EstimatorMode;
using ids_core::IdsConfig;
using ids_core::InputSource;
using ids_ros::configFromParameters;
using ids_ros::defaultParameters;
using ids_ros::ParameterMap;

namespace {

std::string fingerprint(const IdsConfig& c) { return ids_core::configSignature(c); }

}  // namespace

TEST(Parameters, EveryConfigurationFieldHasOneParameter) {
  EXPECT_EQ(defaultParameters().size(), 39u);
}

TEST(Parameters, DefaultsRoundTripToTheDefaultConfiguration) {
  const IdsConfig d;
  const IdsConfig c = configFromParameters(defaultParameters());
  EXPECT_EQ(fingerprint(c), fingerprint(d));
  EXPECT_EQ(c.max_steps_per_window, d.max_steps_per_window);
  EXPECT_DOUBLE_EQ(c.context_guard, d.context_guard);
  EXPECT_DOUBLE_EQ(c.ekf.p0_position, d.ekf.p0_position);
  EXPECT_DOUBLE_EQ(c.ekf.p0_velocity, d.ekf.p0_velocity);
  EXPECT_DOUBLE_EQ(c.ekf.p0_friction, d.ekf.p0_friction);
  EXPECT_DOUBLE_EQ(c.ekf.friction_max, d.ekf.friction_max);
  EXPECT_DOUBLE_EQ(c.ekf.nis_gate, d.ekf.nis_gate);
  EXPECT_EQ(c.ekf.rollback_steps, d.ekf.rollback_steps);
  EXPECT_EQ(c.detector.latch, d.detector.latch);
  EXPECT_DOUBLE_EQ(c.detector.sigma_floor, d.detector.sigma_floor);
  EXPECT_LT((c.model.nominal_payload.inertia - d.model.nominal_payload.inertia).norm(), 1e-15);
}

TEST(Parameters, OverridesReachTheConfiguration) {
  ParameterMap p = defaultParameters();
  p["input_source"] = std::string("commanded_torque");
  p["estimator_mode"] = std::string("thesis");
  p["model.kp"] = std::vector<double>{700, 700, 500, 500, 200, 100, 40};
  p["model.payload_inertia"] = std::vector<double>{0.01, 0.02, 0.03};
  p["detector.debounce"] = std::int64_t{3};
  p["ewma.tau_slow"] = std::int64_t{3};
  const IdsConfig c = configFromParameters(p);
  EXPECT_EQ(c.input_source, InputSource::kCommandedTorque);
  EXPECT_EQ(c.mode, EstimatorMode::kThesis);
  EXPECT_DOUBLE_EQ(c.model.gains.kp(2), 500.0);
  EXPECT_DOUBLE_EQ(c.model.gains.kp(6), 40.0);
  EXPECT_DOUBLE_EQ(c.model.nominal_payload.inertia(1, 1), 0.02);
  EXPECT_DOUBLE_EQ(c.model.nominal_payload.inertia(0, 1), 0.0);
  EXPECT_EQ(c.detector.debounce, 3);
  EXPECT_DOUBLE_EQ(c.ewma.tau_slow, 3.0);
  EXPECT_NE(fingerprint(c), fingerprint(IdsConfig{}));
}

TEST(Parameters, PartialMapsKeepTheOtherDefaults) {
  const IdsConfig c = configFromParameters({{"detector.latch", false}});
  EXPECT_FALSE(c.detector.latch);
  EXPECT_EQ(c.detector.debounce, IdsConfig{}.detector.debounce);
  EXPECT_EQ(c.input_source, InputSource::kTrustedReference);
}

TEST(Parameters, InvalidValuesAreRejected) {
  EXPECT_THROW(configFromParameters({{"input_source", std::string("spoofed")}}),
               std::invalid_argument);
  EXPECT_THROW(configFromParameters({{"estimator_mode", std::string("rls")}}),
               std::invalid_argument);
  EXPECT_THROW(configFromParameters({{"model.kd", std::vector<double>{1.0, 2.0}}}),
               std::invalid_argument);
  EXPECT_THROW(configFromParameters({{"detector.debounce", 2.5}}), std::invalid_argument);
  EXPECT_THROW(configFromParameters({{"detector.debouce", std::int64_t{2}}}),
               std::invalid_argument);
  EXPECT_THROW(configFromParameters({{"ewma.tau_slow", 0.01}}), std::invalid_argument);
  EXPECT_THROW(configFromParameters({{"detector.hysteresis", 1.5}}), std::invalid_argument);
  EXPECT_THROW(configFromParameters({{"model.controller_period", 0.02}}), std::invalid_argument);
  EXPECT_THROW(configFromParameters({{"detector.latch", std::int64_t{1}}}), std::invalid_argument);
}