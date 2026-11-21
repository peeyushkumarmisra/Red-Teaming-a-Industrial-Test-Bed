#include "ids_core/calibration.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include <pinocchio/algorithm/kinematics.hpp>
#include <yaml-cpp/yaml.h>

#include "ids_core/dual_ewma.hpp"
#include "ids_core/limit_selection.hpp"
#include "ids_core/sha256.hpp"
#include "iiwa_control_law/robot_model.hpp"

namespace ids_core {

namespace {

void appendVec(std::ostringstream& out, const Vec7& v) {
  for (int i = 0; i < kDof; ++i) {
    out << v(i) << ',';
  }
}

std::vector<double> toList(const Vec7& v) { return std::vector<double>(v.data(), v.data() + kDof); }

Vec7 fromNode(const YAML::Node& node, const std::string& key) {
  const auto list = node[key].as<std::vector<double>>();
  if (list.size() != static_cast<std::size_t>(kDof)) {
    throw std::runtime_error("calibration field has wrong length: " + key);
  }
  return Eigen::Map<const Vec7>(list.data());
}

bool slowEnabled(const IdsConfig& c) {
  return c.mode == EstimatorMode::kExtended && c.detector.use_slow_detector;
}

bool commandEnabled(const IdsConfig& c) {
  return c.mode == EstimatorMode::kExtended && c.detector.use_command_check;
}

}  // namespace

std::string modelSignature(const std::string& urdf, const std::string& tip_joint) {
  const iiwa_control_law::RobotModel robot =
      iiwa_control_law::RobotModel::fromUrdfString(urdf, tip_joint);
  const pinocchio::Model& m = robot.model();
  pinocchio::Data d(m);
  pinocchio::forwardKinematics(m, d, Eigen::VectorXd::Zero(m.nq));
  std::ostringstream out;
  out << std::setprecision(17) << tip_joint << ';';
  for (pinocchio::JointIndex i = 1; i < static_cast<pinocchio::JointIndex>(m.njoints); ++i) {
    out << m.names[i] << ':';
    const Eigen::MatrixXd s = d.joints[i].S().matrix();
    for (Eigen::Index r = 0; r < s.rows(); ++r) {
      for (Eigen::Index k = 0; k < s.cols(); ++k) {
        out << s(r, k) << ',';
      }
    }
    const auto& placement = m.jointPlacements[i];
    for (int r = 0; r < 3; ++r) {
      out << placement.translation()(r) << ',';
      for (int k = 0; k < 3; ++k) {
        out << placement.rotation()(r, k) << ',';
      }
    }
    const auto& inertia = m.inertias[i];
    const Eigen::Matrix3d rot = inertia.inertia().matrix();
    out << inertia.mass() << ',' << inertia.lever()(0) << ',' << inertia.lever()(1) << ','
        << inertia.lever()(2) << ',' << rot(0, 0) << ',' << rot(0, 1) << ',' << rot(0, 2) << ','
        << rot(1, 1) << ',' << rot(1, 2) << ',' << rot(2, 2) << ',';
    const int q = m.joints[i].idx_q();
    out << m.effortLimit[q] << ',' << m.velocityLimit[q] << ',' << m.lowerPositionLimit[q] << ','
        << m.upperPositionLimit[q] << ';';
  }
  return out.str();
}

std::string configSignature(const IdsConfig& c) {
  std::ostringstream out;
  out << std::setprecision(12);
  out << static_cast<int>(c.input_source) << ',' << static_cast<int>(c.mode) << ','
      << c.nominal_rate_hz << ',';
  out << c.model.controller_period << ',' << c.model.reference_hold << ','
      << c.model.reference_latency << ',' << c.model.tip_joint << ',';
  appendVec(out, c.model.gains.kp);
  appendVec(out, c.model.gains.kd);
  appendVec(out, c.model.gains.effort_limit);
  out << c.model.nominal_payload.mass << ',' << c.model.nominal_payload.com.transpose() << ','
      << c.model.nominal_payload.inertia.diagonal().transpose() << ',';
  out << c.ekf.q_position << ',' << c.ekf.q_velocity << ',' << c.ekf.q_friction << ','
      << c.ekf.r_position << ',' << c.ekf.r_velocity << ',' << c.ekf.refine_iterations << ',';
  out << c.ewma.tau_fast << ',' << c.ewma.tau_slow << ',';
  out << c.detector.false_alarm_budget_per_hour << ',' << c.detector.debounce << ','
      << c.detector.clear_count << ',' << c.detector.hysteresis << ','
      << c.detector.use_slow_detector << ',' << c.detector.use_command_check << ','
      << c.detector.limit_override << ',' << c.detector.slow_limit_override << ','
      << c.context_guard;
  return out.str();
}

std::string computeFingerprint(const std::string& urdf, const IdsConfig& c) {
  return sha256Hex(modelSignature(urdf, c.model.tip_joint) + '|' + configSignature(c));
}

void saveCalibration(const Calibration& cal, const std::string& path) {
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "fingerprint" << YAML::Value << cal.fingerprint;
  out << YAML::Key << "duration_s" << YAML::Value << cal.duration_s;
  out << YAML::Key << "samples" << YAML::Value << cal.samples;
  out << YAML::Key << "limit" << YAML::Value << cal.limit;
  out << YAML::Key << "slow_limit" << YAML::Value << cal.slow_limit;
  out << YAML::Key << "physics_mean" << YAML::Value << YAML::Flow << toList(cal.physics_mean);
  out << YAML::Key << "physics_std" << YAML::Value << YAML::Flow << toList(cal.physics_std);
  out << YAML::Key << "command_mean" << YAML::Value << YAML::Flow << toList(cal.command_mean);
  out << YAML::Key << "command_std" << YAML::Value << YAML::Flow << toList(cal.command_std);
  out << YAML::Key << "physics_scale" << YAML::Value << YAML::Flow << toList(cal.physics_scale);
  out << YAML::Key << "slow_scale" << YAML::Value << YAML::Flow << toList(cal.slow_scale);
  out << YAML::Key << "command_scale" << YAML::Value << YAML::Flow << toList(cal.command_scale);
  out << YAML::Key << "friction_initial" << YAML::Value << YAML::Flow
      << toList(cal.friction_initial);
  out << YAML::EndMap;
  std::ofstream file(path);
  if (!file) {
    throw std::runtime_error("cannot write calibration file: " + path);
  }
  file << out.c_str() << '\n';
}

Calibration loadCalibration(const std::string& path) {
  const YAML::Node node = YAML::LoadFile(path);
  Calibration cal;
  cal.fingerprint = node["fingerprint"].as<std::string>();
  cal.duration_s = node["duration_s"].as<double>();
  cal.samples = node["samples"].as<long>();
  cal.limit = node["limit"].as<double>();
  cal.slow_limit = node["slow_limit"].as<double>();
  cal.physics_mean = fromNode(node, "physics_mean");
  cal.physics_std = fromNode(node, "physics_std");
  cal.command_mean = fromNode(node, "command_mean");
  cal.command_std = fromNode(node, "command_std");
  cal.physics_scale = fromNode(node, "physics_scale");
  cal.slow_scale = fromNode(node, "slow_scale");
  cal.command_scale = fromNode(node, "command_scale");
  cal.friction_initial = fromNode(node, "friction_initial");
  return cal;
}

void CalibrationRecorder::clear() {
  physics_.clear();
  command_.clear();
}

void CalibrationRecorder::add(const Vec7& physics, const Vec7* command) {
  physics_.push_back(physics);
  if (command != nullptr) {
    command_.push_back(*command);
  }
}

namespace {

void moments(const std::vector<Vec7>& data, double floor, Vec7& mean, Vec7& stddev) {
  mean.setZero();
  stddev = Vec7::Constant(floor);
  if (data.size() < 2) {
    return;
  }
  for (const auto& x : data) {
    mean += x;
  }
  mean /= static_cast<double>(data.size());
  Vec7 var = Vec7::Zero();
  for (const auto& x : data) {
    var += (x - mean).cwiseAbs2();
  }
  var /= static_cast<double>(data.size() - 1);
  stddev = var.cwiseSqrt().cwiseMax(floor);
}

void scales(const std::vector<Vec7>& data, const Vec7& mean, const Vec7& stddev,
            const IdsConfig& config, Vec7& diff_scale, Vec7& slow_scale) {
  diff_scale = Vec7::Ones();
  slow_scale = Vec7::Ones();
  DualEwma ewma;
  const double dt = 1.0 / config.nominal_rate_hz;
  ewma.configure(dt, config.ewma.tau_fast, config.ewma.tau_slow);
  const auto skip = static_cast<std::size_t>(2.0 * config.ewma.tau_slow / dt);
  if (data.size() <= skip + 10) {
    return;
  }
  Vec7 sd = Vec7::Zero();
  Vec7 ss = Vec7::Zero();
  std::size_t n = 0;
  for (std::size_t k = 0; k < data.size(); ++k) {
    ewma.update((data[k] - mean).cwiseQuotient(stddev), false);
    if (k >= skip) {
      sd += (ewma.difference() / ewma.sigmaDifference()).cwiseAbs2();
      ss += (ewma.slow() / ewma.sigmaSlow()).cwiseAbs2();
      ++n;
    }
  }
  diff_scale = (sd / static_cast<double>(n)).cwiseSqrt().cwiseMax(1.0);
  slow_scale = (ss / static_cast<double>(n)).cwiseSqrt().cwiseMax(1.0);
}

}  // namespace

Calibration CalibrationRecorder::finalize(const IdsConfig& config, const std::string& fingerprint,
                                          const Vec7& friction, double duration_s) const {
  Calibration cal;
  cal.fingerprint = fingerprint;
  cal.duration_s = duration_s;
  cal.samples = samples();
  cal.friction_initial = friction;
  const double floor = config.detector.sigma_floor;
  moments(physics_, floor, cal.physics_mean, cal.physics_std);
  moments(command_, floor, cal.command_mean, cal.command_std);
  scales(physics_, cal.physics_mean, cal.physics_std, config, cal.physics_scale, cal.slow_scale);
  Vec7 unused;
  scales(command_, cal.command_mean, cal.command_std, config, cal.command_scale, unused);
  int channels = kDof;
  if (slowEnabled(config)) channels += kDof;
  if (commandEnabled(config)) channels += kDof;
  const double target = config.detector.false_alarm_budget_per_hour / channels;
  AlarmModel model;
  model.rate_hz = config.nominal_rate_hz;
  model.tau_fast = config.ewma.tau_fast;
  model.tau_slow = config.ewma.tau_slow;
  model.debounce = config.detector.debounce;
  model.clear_count = config.detector.clear_count;
  model.hysteresis = config.detector.hysteresis;
  model.kind = StatisticKind::kDifference;
  cal.limit = config.detector.limit_override > 0.0 ? config.detector.limit_override
                                                    : selectLimit(model, target, 20.0, 7);
  model.kind = StatisticKind::kSlow;
  cal.slow_limit = config.detector.slow_limit_override > 0.0
                       ? config.detector.slow_limit_override
                       : selectLimit(model, target, 20.0, 11);
  return cal;
}

}  // namespace ids_core