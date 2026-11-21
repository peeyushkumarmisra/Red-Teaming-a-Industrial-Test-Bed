#pragma once

#include <string>
#include <utility>

#include "ids_ros/parameters.hpp"

namespace ids_ros {

ParameterMap parametersFromFile(const std::string& path);
std::pair<std::string, ParameterValue> parseOverride(const std::string& assignment);

}  // namespace ids_ros