// =============================================================================
//  parameter_file.hpp - IDS settings from the node's YAML file
// -----------------------------------------------------------------------------
//  Lets ids_replay read config/ids.yaml or config/ids_thesis.yaml directly, so an
//  offline replay uses exactly the settings of the live node, and lets single settings
//  be overridden for experiments (for example input_source=commanded_torque).
// =============================================================================

#pragma once

#include <string>
#include <utility>

#include "ids_ros/parameters.hpp"

namespace ids_ros {

// Reads ros__parameters (at the top or under a node name), flattens nested keys to dotted names
// and keeps only IDS parameters, each converted to the type the node declares.
ParameterMap parametersFromFile(const std::string& path);
// Parses name=value with the parameter's own type; unknown names and bad values are errors.
std::pair<std::string, ParameterValue> parseOverride(const std::string& assignment);

}  // namespace ids_ros
