#include <gtest/gtest.h>

#include <array>
#include <cstdint>

#include "ids_ros/conversions.hpp"

namespace {

struct FakeReference {
  std::uint64_t seq = 0;
  std::array<double, 7> position{};
  std::array<double, 7> velocity{};
  std::array<double, 7> acceleration{};
  std::uint8_t payload_context = 0;
  double payload_mass = 0.0;
};

struct FakeMeasurement {
  std::array<double, 7> position{};
  std::array<double, 7> velocity{};
  std::array<double, 7> effort_applied{};
  std::uint64_t reference_seq = 0;
  bool halted = false;
};

struct FakeStatus {
  std::uint8_t mode = 0;
  bool alarm = false;
  bool latched = false;
  std::int8_t primary_joint = 0;
  std::array<bool, 7> physics_alarm{};
  std::array<bool, 7> command_alarm{};
  std::array<double, 7> disturbance{};
  std::array<double, 7> command_error{};
  std::array<double, 7> physics_statistic{};
  std::array<double, 7> physics_slow_statistic{};
  std::array<double, 7> command_statistic{};
  std::array<double, 7> friction_estimate{};
  double limit = 0.0;
  double nis = 0.0;
  double step_time_us = 0.0;
};

}  // namespace

TEST(Conversions, ReferenceMessageBecomesASample) {
  FakeReference msg;
  msg.seq = 42;
  for (std::size_t i = 0; i < 7; ++i) {
    msg.position[i] = 0.1 * static_cast<double>(i);
    msg.velocity[i] = -0.2 * static_cast<double>(i);
    msg.acceleration[i] = 0.3;
  }
  msg.payload_context = 1;
  msg.payload_mass = 5.0;
  const auto s = ids_ros::toReferenceSample(msg, 12.34);
  EXPECT_DOUBLE_EQ(s.stamp, 12.34);
  EXPECT_EQ(s.seq, 42u);
  EXPECT_DOUBLE_EQ(s.position(6), 0.6);
  EXPECT_DOUBLE_EQ(s.velocity(3), -0.6);
  EXPECT_DOUBLE_EQ(s.acceleration(0), 0.3);
  EXPECT_EQ(s.payload_context, 1);
  EXPECT_DOUBLE_EQ(s.payload_mass, 5.0);
}

TEST(Conversions, MeasurementMessageBecomesAMeasurement) {
  FakeMeasurement msg;
  msg.position[2] = 1.5;
  msg.velocity[4] = -0.5;
  msg.effort_applied[1] = 120.0;
  msg.reference_seq = 7;
  msg.halted = true;
  const auto m = ids_ros::toMeasurement(msg, 3.0);
  EXPECT_DOUBLE_EQ(m.stamp, 3.0);
  EXPECT_DOUBLE_EQ(m.q(2), 1.5);
  EXPECT_DOUBLE_EQ(m.v(4), -0.5);
  EXPECT_DOUBLE_EQ(m.tau(1), 120.0);
  EXPECT_EQ(m.reference_seq, 7u);
  EXPECT_TRUE(m.halted);
}

TEST(Conversions, OutputFillsEveryStatusField) {
  ids_core::IdsOutput out;
  out.mode = ids_core::Mode::kArmed;
  out.alarm = true;
  out.latched = true;
  out.primary_joint = 4;
  out.physics_alarm(4) = true;
  out.command_alarm(4) = true;
  out.disturbance(4) = 5.0;
  out.command_error(4) = 6.0;
  out.physics_statistic(4) = 7.0;
  out.physics_slow_statistic(4) = 8.0;
  out.command_statistic(4) = 9.0;
  out.friction(0) = 4.1;
  out.limit = 4.4;
  out.nis = 13.0;
  FakeStatus msg;
  ids_ros::fillStatus(out, 215.0, msg);
  EXPECT_EQ(msg.mode, 2);
  EXPECT_TRUE(msg.alarm);
  EXPECT_TRUE(msg.latched);
  EXPECT_EQ(msg.primary_joint, 4);
  EXPECT_TRUE(msg.physics_alarm[4]);
  EXPECT_FALSE(msg.physics_alarm[3]);
  EXPECT_TRUE(msg.command_alarm[4]);
  EXPECT_DOUBLE_EQ(msg.disturbance[4], 5.0);
  EXPECT_DOUBLE_EQ(msg.command_error[4], 6.0);
  EXPECT_DOUBLE_EQ(msg.physics_statistic[4], 7.0);
  EXPECT_DOUBLE_EQ(msg.physics_slow_statistic[4], 8.0);
  EXPECT_DOUBLE_EQ(msg.command_statistic[4], 9.0);
  EXPECT_DOUBLE_EQ(msg.friction_estimate[0], 4.1);
  EXPECT_DOUBLE_EQ(msg.limit, 4.4);
  EXPECT_DOUBLE_EQ(msg.nis, 13.0);
  EXPECT_DOUBLE_EQ(msg.step_time_us, 215.0);
}

TEST(Conversions, NoAlarmMeansNoPrimaryJoint) {
  ids_core::IdsOutput out;
  FakeStatus msg;
  ids_ros::fillStatus(out, 0.0, msg);
  EXPECT_EQ(msg.mode, 0);
  EXPECT_EQ(msg.primary_joint, -1);
}