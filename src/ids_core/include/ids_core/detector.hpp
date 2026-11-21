#pragma once

#include <Eigen/Core>

#include "ids_core/config.hpp"
#include "ids_core/types.hpp"

namespace ids_core {

struct DetectorOutput {
  bool alarm = false;
  bool latched = false;
  bool onset = false;
  Mask7 physics = Mask7::Constant(false);
  Mask7 physics_slow = Mask7::Constant(false);
  Mask7 command = Mask7::Constant(false);
};

class ChannelBank {
 public:
  void reset();
  void step(const Vec7& statistic, double limit, int debounce, int clear_count, double hysteresis);
  const Mask7& active() const { return active_; }

 private:
  Eigen::Array<int, kDof, 1> over_ = Eigen::Array<int, kDof, 1>::Zero();
  Eigen::Array<int, kDof, 1> under_ = Eigen::Array<int, kDof, 1>::Zero();
  Mask7 active_ = Mask7::Constant(false);
};

class Detector {
 public:
  void configure(const DetectorConfig& config, double limit, double slow_limit);
  void reset();
  const DetectorOutput& update(const Vec7& physics, const Vec7& physics_slow, const Vec7& command,
                               bool use_slow, bool use_command);
  const DetectorOutput& output() const { return out_; }
  double limit() const { return limit_; }
  double slowLimit() const { return slow_limit_; }

 private:
  DetectorConfig config_;
  double limit_ = 4.0;
  double slow_limit_ = 4.0;
  ChannelBank physics_;
  ChannelBank slow_;
  ChannelBank command_;
  DetectorOutput out_;
};

}  // namespace ids_core