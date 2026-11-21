// =============================================================================
//  types.hpp - fixed-size vectors for a 7-joint arm
// -----------------------------------------------------------------------------
//  Fixed-size Eigen types (no heap allocation), shared by the controller and the IDS.
// =============================================================================

#pragma once

#include <Eigen/Core>

namespace iiwa_control_law {

constexpr int kDof = 7;

using Vec7 = Eigen::Matrix<double, kDof, 1>;
using Mat7 = Eigen::Matrix<double, kDof, kDof>;
using Mask7 = Eigen::Matrix<bool, kDof, 1>;

}  // namespace iiwa_control_law
