// =============================================================================
//  encoder_model.hpp - what a real joint sensor would report
// -----------------------------------------------------------------------------
//  Positions are rounded to a 20-bit encoder step (about 6e-6 rad) and velocities get
//  seeded Gaussian noise. Why it matters: Gazebo's joint states are perfect; without
//  sensor noise the IDS would be tested against an unrealistically clean signal.
//  The run's seed changes the noise, so repeated runs are independent.
// =============================================================================

#pragma once

#include <cmath>
#include <cstdint>
#include <numbers>
#include <random>

#include "iiwa_control_law/types.hpp"

namespace iiwa_controllers {

// Resolution, velocity noise and seed.
struct EncoderConfig {
  int position_bits = 20;
  double velocity_noise = 1e-3;
  std::uint64_t seed = 1;
};

class EncoderModel {
 public:
  explicit EncoderModel(const EncoderConfig& config)
      : resolution_(config.position_bits > 0
                        ? 2.0 * std::numbers::pi / std::ldexp(1.0, config.position_bits)
                        : 0.0),
        noise_(config.velocity_noise),
        rng_(config.seed),
        normal_(0.0, 1.0) {}

  double resolution() const { return resolution_; }

  // Turn the true joint state into the measured one.
  void sense(const iiwa_control_law::Vec7& q, const iiwa_control_law::Vec7& v,
             iiwa_control_law::Vec7& qm, iiwa_control_law::Vec7& vm) {
    for (int i = 0; i < iiwa_control_law::kDof; ++i) {
      qm(i) = resolution_ > 0.0 ? std::round(q(i) / resolution_) * resolution_ : q(i);
      vm(i) = noise_ > 0.0 ? v(i) + noise_ * normal_(rng_) : v(i);
    }
  }

 private:
  double resolution_;
  double noise_;
  std::mt19937_64 rng_;
  std::normal_distribution<double> normal_;
};

}  // namespace iiwa_controllers
