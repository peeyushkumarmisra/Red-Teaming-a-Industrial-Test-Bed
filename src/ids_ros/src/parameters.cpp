// =============================================================================
//  parameters.cpp - the parameter table and its validation
// -----------------------------------------------------------------------------
//  One table maps each parameter name to the IdsConfig field it reads and writes, so
//  defaults, overrides and the test "every field has one parameter" all use the same
//  list.
// =============================================================================

#include "ids_ros/parameters.hpp"

#include <algorithm>
#include <functional>
#include <stdexcept>

#include "ids_core/types.hpp"

namespace ids_ros {

namespace {

using ids_core::EstimatorMode;
using ids_core::IdsConfig;
using ids_core::InputSource;
using ids_core::Vec7;

// One parameter: its name, how to read it from the config and how to write it.
struct Binding {
  std::string name;
  std::function<ParameterValue(const IdsConfig&)> get;
  std::function<void(IdsConfig&, const ParameterValue&)> set;
};

// Accept integers where numbers are expected (YAML writes 1 for 1.0).
double asDouble(const std::string& name, const ParameterValue& v) {
  if (const auto* d = std::get_if<double>(&v)) return *d;
  if (const auto* i = std::get_if<std::int64_t>(&v)) return static_cast<double>(*i);
  throw std::invalid_argument("parameter " + name + " must be a number");
}

std::int64_t asInt(const std::string& name, const ParameterValue& v) {
  if (const auto* i = std::get_if<std::int64_t>(&v)) return *i;
  throw std::invalid_argument("parameter " + name + " must be an integer");
}

bool asBool(const std::string& name, const ParameterValue& v) {
  if (const auto* b = std::get_if<bool>(&v)) return *b;
  throw std::invalid_argument("parameter " + name + " must be a boolean");
}

std::string asString(const std::string& name, const ParameterValue& v) {
  if (const auto* s = std::get_if<std::string>(&v)) return *s;
  throw std::invalid_argument("parameter " + name + " must be a string");
}

std::vector<double> asList(const std::string& name, const ParameterValue& v, std::size_t size) {
  const auto* l = std::get_if<std::vector<double>>(&v);
  if (l == nullptr || l->size() != size) {
    throw std::invalid_argument("parameter " + name + " must be a list of " + std::to_string(size) +
                                " numbers");
  }
  return *l;
}

std::vector<double> listOf(const Vec7& v) { return {v.data(), v.data() + ids_core::kDof}; }

Vec7 vec7Of(const std::vector<double>& l) { return Eigen::Map<const Vec7>(l.data()); }

template <class F>
Binding real(const std::string& name, F field) {
  return {name, [field](const IdsConfig& c) -> ParameterValue { return field(c); },
          [field, name](IdsConfig& c, const ParameterValue& v) { field(c) = asDouble(name, v); }};
}

template <class F>
Binding integer(const std::string& name, F field) {
  return {name,
          [field](const IdsConfig& c) -> ParameterValue {
            return static_cast<std::int64_t>(field(c));
          },
          [field, name](IdsConfig& c, const ParameterValue& v) {
            field(c) = static_cast<int>(asInt(name, v));
          }};
}

template <class F>
Binding flag(const std::string& name, F field) {
  return {name, [field](const IdsConfig& c) -> ParameterValue { return field(c); },
          [field, name](IdsConfig& c, const ParameterValue& v) { field(c) = asBool(name, v); }};
}

template <class F>
Binding vector7(const std::string& name, F field) {
  return {name, [field](const IdsConfig& c) -> ParameterValue { return listOf(field(c)); },
          [field, name](IdsConfig& c, const ParameterValue& v) {
            field(c) = vec7Of(asList(name, v, ids_core::kDof));
          }};
}

#define IDS_FIELD(member) [](auto& c) -> auto& { return c.member; }

// The full table: every IDS setting, grouped like config.hpp.
const std::vector<Binding>& bindings() {
  static const std::vector<Binding> table = {
      {"input_source",
       [](const IdsConfig& c) -> ParameterValue {
         return std::string(c.input_source == InputSource::kTrustedReference ? "trusted_reference"
                                                                             : "commanded_torque");
       },
       [](IdsConfig& c, const ParameterValue& v) {
         const std::string s = asString("input_source", v);
         if (s == "trusted_reference") {
           c.input_source = InputSource::kTrustedReference;
         } else if (s == "commanded_torque") {
           c.input_source = InputSource::kCommandedTorque;
         } else {
           throw std::invalid_argument(
               "input_source must be 'trusted_reference' or 'commanded_torque'");
         }
       }},
      {"estimator_mode",
       [](const IdsConfig& c) -> ParameterValue {
         return std::string(c.mode == EstimatorMode::kExtended ? "extended" : "thesis");
       },
       [](IdsConfig& c, const ParameterValue& v) {
         const std::string s = asString("estimator_mode", v);
         if (s == "extended") {
           c.mode = EstimatorMode::kExtended;
         } else if (s == "thesis") {
           c.mode = EstimatorMode::kThesis;
         } else {
           throw std::invalid_argument("estimator_mode must be 'extended' or 'thesis'");
         }
       }},
      real("nominal_rate_hz", IDS_FIELD(nominal_rate_hz)),
      integer("max_steps_per_window", IDS_FIELD(max_steps_per_window)),
      real("context_guard", IDS_FIELD(context_guard)),
      real("model.controller_period", IDS_FIELD(model.controller_period)),
      real("model.reference_hold", IDS_FIELD(model.reference_hold)),
      real("model.reference_latency", IDS_FIELD(model.reference_latency)),
      {"model.tip_joint",
       [](const IdsConfig& c) -> ParameterValue { return c.model.tip_joint; },
       [](IdsConfig& c, const ParameterValue& v) {
         c.model.tip_joint = asString("model.tip_joint", v);
       }},
      vector7("model.kp", IDS_FIELD(model.gains.kp)),
      vector7("model.kd", IDS_FIELD(model.gains.kd)),
      vector7("model.effort_limit", IDS_FIELD(model.gains.effort_limit)),
      real("model.payload_mass", IDS_FIELD(model.nominal_payload.mass)),
      {"model.payload_com",
       [](const IdsConfig& c) -> ParameterValue {
         const auto& p = c.model.nominal_payload.com;
         return std::vector<double>{p.x(), p.y(), p.z()};
       },
       [](IdsConfig& c, const ParameterValue& v) {
         const auto l = asList("model.payload_com", v, 3);
         c.model.nominal_payload.com = Eigen::Vector3d(l[0], l[1], l[2]);
       }},
      {"model.payload_inertia",
       [](const IdsConfig& c) -> ParameterValue {
         const auto d = c.model.nominal_payload.inertia.diagonal();
         return std::vector<double>{d.x(), d.y(), d.z()};
       },
       [](IdsConfig& c, const ParameterValue& v) {
         const auto l = asList("model.payload_inertia", v, 3);
         c.model.nominal_payload.inertia = Eigen::Vector3d(l[0], l[1], l[2]).asDiagonal();
       }},
      real("ekf.q_position", IDS_FIELD(ekf.q_position)),
      real("ekf.q_velocity", IDS_FIELD(ekf.q_velocity)),
      real("ekf.q_friction", IDS_FIELD(ekf.q_friction)),
      real("ekf.r_position", IDS_FIELD(ekf.r_position)),
      real("ekf.r_velocity", IDS_FIELD(ekf.r_velocity)),
      real("ekf.p0_position", IDS_FIELD(ekf.p0_position)),
      real("ekf.p0_velocity", IDS_FIELD(ekf.p0_velocity)),
      real("ekf.p0_friction", IDS_FIELD(ekf.p0_friction)),
      real("ekf.friction_max", IDS_FIELD(ekf.friction_max)),
      real("ekf.nis_gate", IDS_FIELD(ekf.nis_gate)),
      integer("ekf.rollback_steps", IDS_FIELD(ekf.rollback_steps)),
      integer("ekf.refine_iterations", IDS_FIELD(ekf.refine_iterations)),
      real("ewma.tau_fast", IDS_FIELD(ewma.tau_fast)),
      real("ewma.tau_slow", IDS_FIELD(ewma.tau_slow)),
      real("detector.false_alarm_budget_per_hour",
           IDS_FIELD(detector.false_alarm_budget_per_hour)),
      integer("detector.debounce", IDS_FIELD(detector.debounce)),
      integer("detector.clear_count", IDS_FIELD(detector.clear_count)),
      real("detector.hysteresis", IDS_FIELD(detector.hysteresis)),
      flag("detector.latch", IDS_FIELD(detector.latch)),
      flag("detector.use_slow_detector", IDS_FIELD(detector.use_slow_detector)),
      flag("detector.use_command_check", IDS_FIELD(detector.use_command_check)),
      real("detector.limit_override", IDS_FIELD(detector.limit_override)),
      real("detector.slow_limit_override", IDS_FIELD(detector.slow_limit_override)),
      real("detector.sigma_floor", IDS_FIELD(detector.sigma_floor)),
  };
  return table;
}

#undef IDS_FIELD

void require(bool ok, const char* what) {
  if (!ok) {
    throw std::invalid_argument(what);
  }
}

}  // namespace

// Defaults declared by the node, taken from IdsConfig itself (one source of truth).
ParameterMap defaultParameters(const IdsConfig& defaults) {
  ParameterMap out;
  for (const auto& b : bindings()) {
    out.emplace(b.name, b.get(defaults));
  }
  return out;
}

// Apply overrides by name; an unknown name is an error, not ignored.
IdsConfig configFromParameters(const ParameterMap& values) {
  IdsConfig c;
  const auto& table = bindings();
  for (const auto& [name, value] : values) {
    const auto it = std::find_if(table.begin(), table.end(),
                                 [&name](const Binding& b) { return b.name == name; });
    if (it == table.end()) {
      throw std::invalid_argument("unknown IDS parameter: " + name);
    }
    it->set(c, value);
  }
  validateConfig(c);
  return c;
}

// Reject settings that would make the model or the statistics meaningless.
void validateConfig(const IdsConfig& c) {
  require(c.nominal_rate_hz > 0.0, "nominal_rate_hz must be positive");
  require(c.max_steps_per_window >= 1, "max_steps_per_window must be at least 1");
  require(c.context_guard >= 0.0, "context_guard must not be negative");
  require(c.model.controller_period > 0.0 &&
              c.model.controller_period <= 1.0 / c.nominal_rate_hz + 1e-12,
          "model.controller_period must be positive and not longer than one IDS window");
  require(c.model.reference_hold >= 0.0, "model.reference_hold must not be negative");
  require(c.model.reference_latency >= 0.0, "model.reference_latency must not be negative");
  require((c.model.gains.kp.array() > 0.0).all(), "model.kp must be positive");
  require((c.model.gains.kd.array() >= 0.0).all(), "model.kd must not be negative");
  require((c.model.gains.effort_limit.array() > 0.0).all(), "model.effort_limit must be positive");
  require(c.model.nominal_payload.mass >= 0.0, "model.payload_mass must not be negative");
  require((c.model.nominal_payload.inertia.diagonal().array() >= 0.0).all(),
          "model.payload_inertia must not be negative");
  require(c.ekf.q_position >= 0.0 && c.ekf.q_velocity > 0.0 && c.ekf.q_friction >= 0.0,
          "EKF process noise must be positive");
  require(c.ekf.r_position > 0.0 && c.ekf.r_velocity > 0.0,
          "EKF measurement noise must be positive");
  require(c.ekf.p0_position > 0.0 && c.ekf.p0_velocity > 0.0 && c.ekf.p0_friction >= 0.0,
          "EKF initial covariance must be positive");
  require(c.ekf.friction_max > 0.0, "ekf.friction_max must be positive");
  require(c.ekf.nis_gate > 0.0, "ekf.nis_gate must be positive");
  require(c.ekf.rollback_steps >= 0, "ekf.rollback_steps must not be negative");
  require(c.ekf.refine_iterations >= 0, "ekf.refine_iterations must not be negative");
  require(c.ewma.tau_fast > 0.0 && c.ewma.tau_slow > c.ewma.tau_fast,
          "ewma.tau_slow must be longer than ewma.tau_fast, and both positive");
  require(c.detector.false_alarm_budget_per_hour > 0.0,
          "detector.false_alarm_budget_per_hour must be positive");
  require(c.detector.debounce >= 1, "detector.debounce must be at least 1");
  require(c.detector.clear_count >= 1, "detector.clear_count must be at least 1");
  require(c.detector.hysteresis > 0.0 && c.detector.hysteresis <= 1.0,
          "detector.hysteresis must be in (0, 1]");
  require(c.detector.limit_override >= 0.0 && c.detector.slow_limit_override >= 0.0,
          "detector limit overrides must not be negative");
  require(c.detector.sigma_floor > 0.0, "detector.sigma_floor must be positive");
}

}  // namespace ids_ros
