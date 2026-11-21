// =============================================================================
//  parameter_file.cpp - read the IDS parameter YAML without ROS
// -----------------------------------------------------------------------------
//  The defaults of the node's parameter table give each setting its type, so a value in
//  the file that does not fit (text where a number is expected, a list of the wrong
//  kind) is rejected instead of being guessed.
// =============================================================================

#include "ids_ros/parameter_file.hpp"

#include <stdexcept>
#include <type_traits>

#include <yaml-cpp/yaml.h>

namespace ids_ros {

namespace {

// Convert one YAML value to the declared parameter type.
ParameterValue convert(const YAML::Node& node, const ParameterValue& like,
                       const std::string& name) {
  try {
    return std::visit(
        [&node](const auto& example) -> ParameterValue {
          using T = std::decay_t<decltype(example)>;
          return node.as<T>();
        },
        like);
  } catch (const YAML::Exception&) {
    throw std::invalid_argument("parameter " + name + " has a value of the wrong type");
  }
}

// detector: {debounce: 5} becomes detector.debounce = 5, as ROS names it.
void flatten(const YAML::Node& node, const std::string& prefix, const ParameterMap& defaults,
             ParameterMap& out) {
  for (const auto& item : node) {
    const std::string key = item.first.as<std::string>();
    const std::string name = prefix.empty() ? key : prefix + "." + key;
    if (item.second.IsMap()) {
      flatten(item.second, name, defaults, out);
      continue;
    }
    const auto it = defaults.find(name);
    if (it != defaults.end()) {
      out[name] = convert(item.second, it->second, name);
    }
  }
}

}  // namespace

ParameterMap parametersFromFile(const std::string& path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception& e) {
    throw std::invalid_argument("cannot read " + path + ": " + e.what());
  }
  YAML::Node params;
  if (root.IsMap() && root["ros__parameters"]) {
    params = root["ros__parameters"];
  } else if (root.IsMap()) {
    for (const auto& item : root) {
      if (item.second.IsMap() && item.second["ros__parameters"]) {
        params = item.second["ros__parameters"];
        break;
      }
    }
  }
  if (!params || !params.IsMap()) {
    throw std::invalid_argument(path + " has no ros__parameters section");
  }
  ParameterMap out;
  flatten(params, "", defaultParameters(), out);
  return out;
}

std::pair<std::string, ParameterValue> parseOverride(const std::string& assignment) {
  const auto eq = assignment.find('=');
  if (eq == std::string::npos || eq == 0) {
    throw std::invalid_argument("override must look like name=value: " + assignment);
  }
  const std::string name = assignment.substr(0, eq);
  std::string text = assignment.substr(eq + 1);
  const ParameterMap defaults = defaultParameters();
  const auto it = defaults.find(name);
  if (it == defaults.end()) {
    throw std::invalid_argument("unknown IDS parameter: " + name);
  }
  if (std::holds_alternative<std::vector<double>>(it->second) && text.find('[') == text.npos) {
    text = "[" + text + "]";
  }
  if (std::holds_alternative<std::string>(it->second)) {
    return {name, ParameterValue(text)};
  }
  YAML::Node node;
  try {
    node = YAML::Load(text);
  } catch (const YAML::Exception&) {
    throw std::invalid_argument("cannot parse the value of " + name);
  }
  return {name, convert(node, it->second, name)};
}

}  // namespace ids_ros
