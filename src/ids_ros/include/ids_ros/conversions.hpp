#pragma once

#include <cstddef>
#include <cstdint>

#include "ids_core/ids_pipeline.hpp"
#include "iiwa_control_law/control_law.hpp"

namespace ids_ros {

template <class Array>
ids_core::Vec7 toVec7(const Array& a) {
  ids_core::Vec7 v;
  for (int i = 0; i < ids_core::kDof; ++i) {
    v(i) = a[static_cast<std::size_t>(i)];
  }
  return v;
}

template <class Array>
void fromVec7(const ids_core::Vec7& v, Array& a) {
  for (int i = 0; i < ids_core::kDof; ++i) {
    a[static_cast<std::size_t>(i)] = v(i);
  }
}

template <class Array>
void fromMask7(const ids_core::Mask7& m, Array& a) {
  for (int i = 0; i < ids_core::kDof; ++i) {
    a[static_cast<std::size_t>(i)] = m(i);
  }
}

template <class ReferenceMsg>
iiwa_control_law::ReferenceSample toReferenceSample(const ReferenceMsg& msg, double stamp) {
  iiwa_control_law::ReferenceSample s;
  s.stamp = stamp;
  s.seq = msg.seq;
  s.position = toVec7(msg.position);
  s.velocity = toVec7(msg.velocity);
  s.acceleration = toVec7(msg.acceleration);
  s.payload_context = msg.payload_context;
  s.payload_mass = msg.payload_mass;
  return s;
}

template <class MeasurementMsg>
ids_core::Measurement toMeasurement(const MeasurementMsg& msg, double stamp) {
  ids_core::Measurement m;
  m.stamp = stamp;
  m.q = toVec7(msg.position);
  m.v = toVec7(msg.velocity);
  m.tau = toVec7(msg.effort_applied);
  m.reference_seq = msg.reference_seq;
  m.halted = msg.halted;
  return m;
}

template <class StatusMsg>
void fillStatus(const ids_core::IdsOutput& out, double step_time_us, StatusMsg& msg) {
  msg.mode = static_cast<std::uint8_t>(out.mode);
  msg.alarm = out.alarm;
  msg.latched = out.latched;
  msg.primary_joint = static_cast<std::int8_t>(out.primary_joint);
  fromMask7(out.physics_alarm, msg.physics_alarm);
  fromMask7(out.command_alarm, msg.command_alarm);
  fromVec7(out.disturbance, msg.disturbance);
  fromVec7(out.command_error, msg.command_error);
  fromVec7(out.physics_statistic, msg.physics_statistic);
  fromVec7(out.physics_slow_statistic, msg.physics_slow_statistic);
  fromVec7(out.command_statistic, msg.command_statistic);
  fromVec7(out.friction, msg.friction_estimate);
  msg.limit = out.limit;
  msg.nis = out.nis;
  msg.step_time_us = step_time_us;
}

}  // namespace ids_ros