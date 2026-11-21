// =============================================================================
//  types.hpp - sizes and state of the estimator
// -----------------------------------------------------------------------------
//  The filter state has 21 numbers: joint positions q (7), velocities v (7) and viscous
//  friction coefficients b (7). Each measurement has 14: q and v.
// =============================================================================

#pragma once

#include <Eigen/Core>

#include "iiwa_control_law/types.hpp"

namespace ids_core {

using iiwa_control_law::kDof;
using iiwa_control_law::Mask7;
using iiwa_control_law::Mat7;
using iiwa_control_law::Vec7;

constexpr int kMeas = 2 * kDof;
constexpr int kStates = 3 * kDof;

using Vec14 = Eigen::Matrix<double, kMeas, 1>;
using Mat14 = Eigen::Matrix<double, kMeas, kMeas>;
using Vec21 = Eigen::Matrix<double, kStates, 1>;
using Mat21 = Eigen::Matrix<double, kStates, kStates>;
using Mat21x7 = Eigen::Matrix<double, kStates, kDof>;
using Mat21x14 = Eigen::Matrix<double, kStates, kMeas>;

// Estimator state: where the arm is, how fast it moves, and how much friction each joint has.
struct State {
  Vec7 q = Vec7::Zero();
  Vec7 v = Vec7::Zero();
  Vec7 b = Vec7::Zero();
};

}  // namespace ids_core
