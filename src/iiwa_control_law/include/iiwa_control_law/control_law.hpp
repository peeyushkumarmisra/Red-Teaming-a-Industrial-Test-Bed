#pragma once

#include <cstdint>

#include "iiwa_control_law/types.hpp"

namespace iiwa_control_law {

struct Gains {
  Vec7 kp;
  Vec7 kd;
  Vec7 effort_limit;

  static Gains iiwaDefault() {
    Gains g;
    g.kp << 800.0, 800.0, 600.0, 600.0, 250.0, 150.0, 50.0;
    g.kd << 64.6, 81.6, 32.4, 32.0, 6.4, 3.8, 1.4;
    g.effort_limit << 320.0, 320.0, 176.0, 176.0, 110.0, 40.0, 40.0;
    return g;
  }
};

struct ReferenceSample {
  double stamp = 0.0;
  std::uint64_t seq = 0;
  Vec7 position = Vec7::Zero();
  Vec7 velocity = Vec7::Zero();
  Vec7 acceleration = Vec7::Zero();
  std::uint8_t payload_context = 0;
  double payload_mass = 0.0;
};

struct ReferencePoint {
  Vec7 position;
  Vec7 velocity;
};

inline ReferencePoint referenceAt(const ReferenceSample& sample, double t, double max_hold) {
  const double dt = t - sample.stamp;
  if (dt <= 0.0) {
    return {sample.position, sample.velocity};
  }
  if (dt > max_hold) {
    return {sample.position + sample.velocity * max_hold, Vec7::Zero()};
  }
  return {sample.position + sample.velocity * dt, sample.velocity};
}

inline Vec7 computeTorque(const Gains& gains, const Vec7& q, const Vec7& v,
                          const ReferencePoint& reference, const Vec7& gravity,
                          Mask7* saturated = nullptr) {
  const Vec7 raw = gains.kp.cwiseProduct(reference.position - q) +
                   gains.kd.cwiseProduct(reference.velocity - v) + gravity;
  if (saturated != nullptr) {
    for (int i = 0; i < kDof; ++i) {
      (*saturated)(i) = raw(i) > gains.effort_limit(i) || raw(i) < -gains.effort_limit(i);
    }
  }
  return raw.cwiseMax(-gains.effort_limit).cwiseMin(gains.effort_limit);
}

inline Vec7 holdTorque(const Gains& gains, const Vec7& q, const Vec7& v, const Vec7& hold_position,
                       const Vec7& gravity) {
  return computeTorque(gains, q, v, ReferencePoint{hold_position, Vec7::Zero()}, gravity);
}

}  // namespace iiwa_control_law