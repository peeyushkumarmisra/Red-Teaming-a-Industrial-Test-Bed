// =============================================================================
//  control_law.hpp - the joint control law (PD + gravity feed-forward)
// -----------------------------------------------------------------------------
//  Why it matters: the IDS detects attacks by predicting how the robot SHOULD move
//  when it follows the trusted PLC reference. That prediction is only right if the IDS
//  uses exactly the controller's law, gains and limits. This header is that law, used
//  by both sides.
//  What it does:
//  - holds the reference between 10 ms samples (first-order hold);
//  - computes torque = Kp*(q_ref - q) + Kd*(v_ref - v) + gravity, then clamps it to the
//    joint's effort limit.
//  Used by: iiwa_controllers (the robot) and ids_core (the closed-loop model).
// =============================================================================

#pragma once

#include <cstdint>

#include "iiwa_control_law/types.hpp"

namespace iiwa_control_law {

// Controller gains and torque limits per joint (a1..a7).
struct Gains {
  Vec7 kp;
  Vec7 kd;
  Vec7 effort_limit;

  // Default gains for the KUKA LBR iiwa 14. The controller and the IDS must use the same
  // values. The IDS's gains are part of its calibration fingerprint, so changing them
  // invalidates an old calibration.
  static Gains iiwaDefault() {
    Gains g;
    g.kp << 800.0, 800.0, 600.0, 600.0, 250.0, 150.0, 50.0;
    g.kd << 64.6, 81.6, 32.4, 32.0, 6.4, 3.8, 1.4;
    g.effort_limit << 320.0, 320.0, 176.0, 176.0, 110.0, 40.0, 40.0;
    return g;
  }
};

// One sample of the PLC reference, as received (see JointReference.msg).
struct ReferenceSample {
  double stamp = 0.0;
  std::uint64_t seq = 0;
  Vec7 position = Vec7::Zero();
  Vec7 velocity = Vec7::Zero();
  Vec7 acceleration = Vec7::Zero();
  std::uint8_t payload_context = 0;
  double payload_mass = 0.0;
};

// The reference evaluated at one controller tick.
struct ReferencePoint {
  Vec7 position;
  Vec7 velocity;
};

// First-order hold: between samples, move along the sample's velocity. If the next sample
// is late by more than max_hold, stop extrapolating and hold still (safe behaviour).
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

// The control law. A setpoint attack of gamma rad on joint i changes the torque by
// Kp_i * gamma on that joint only (see test AttackOnSetpointShiftsTorqueByKpGamma).
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
  // Clamp to the motor limits, as the real drive would.
  return raw.cwiseMax(-gains.effort_limit).cwiseMin(gains.effort_limit);
}

// Hold a fixed position (used while waiting, halted or starved of references).
inline Vec7 holdTorque(const Gains& gains, const Vec7& q, const Vec7& v, const Vec7& hold_position,
                       const Vec7& gravity) {
  return computeTorque(gains, q, v, ReferencePoint{hold_position, Vec7::Zero()}, gravity);
}

}  // namespace iiwa_control_law
