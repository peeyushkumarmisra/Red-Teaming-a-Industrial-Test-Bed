// =============================================================================
//  parameters.hpp - ROS parameters <-> IDS configuration
// -----------------------------------------------------------------------------
//  Every IdsConfig field is one ROS parameter with the same dotted name
//  (for example detector.debounce). Unknown names and invalid values are rejected
//  when the node starts, so a typo cannot silently change detection.
// =============================================================================

#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <variant>
#include <vector>

#include "ids_core/config.hpp"

namespace ids_ros {

using ParameterValue = std::variant<bool, std::int64_t, double, std::string, std::vector<double>>;
using ParameterMap = std::map<std::string, ParameterValue>;

ParameterMap defaultParameters(const ids_core::IdsConfig& defaults = ids_core::IdsConfig{});
ids_core::IdsConfig configFromParameters(const ParameterMap& values);
void validateConfig(const ids_core::IdsConfig& config);

}  // namespace ids_ros
