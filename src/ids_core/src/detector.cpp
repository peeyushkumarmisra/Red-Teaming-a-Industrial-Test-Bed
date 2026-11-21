#include "ids_core/detector.hpp"

namespace ids_core {

void ChannelBank::reset() {
  over_.setZero();
  under_.setZero();
  active_.setConstant(false);
}

void ChannelBank::step(const Vec7& statistic, double limit, int debounce, int clear_count,
                       double hysteresis) {
  for (int i = 0; i < kDof; ++i) {
    if (!active_(i)) {
      over_(i) = statistic(i) > limit ? over_(i) + 1 : 0;
      if (over_(i) >= debounce) {
        active_(i) = true;
        under_(i) = 0;
      }
    } else {
      under_(i) = statistic(i) < hysteresis * limit ? under_(i) + 1 : 0;
      if (under_(i) >= clear_count) {
        active_(i) = false;
        over_(i) = 0;
      }
    }
  }
}

void Detector::configure(const DetectorConfig& config, double limit, double slow_limit) {
  config_ = config;
  limit_ = limit;
  slow_limit_ = slow_limit;
  reset();
}

void Detector::reset() {
  physics_.reset();
  slow_.reset();
  command_.reset();
  out_ = DetectorOutput{};
}

const DetectorOutput& Detector::update(const Vec7& physics, const Vec7& physics_slow,
                                       const Vec7& command, bool use_slow, bool use_command) {
  const bool previous = out_.alarm;
  physics_.step(physics, limit_, config_.debounce, config_.clear_count, config_.hysteresis);
  if (use_slow) {
    slow_.step(physics_slow, slow_limit_, config_.debounce, config_.clear_count,
               config_.hysteresis);
  } else {
    slow_.reset();
  }
  if (use_command) {
    command_.step(command, limit_, config_.debounce, config_.clear_count, config_.hysteresis);
  } else {
    command_.reset();
  }
  out_.physics = physics_.active();
  out_.physics_slow = slow_.active();
  out_.command = command_.active();
  out_.alarm = out_.physics.any() || out_.physics_slow.any() || out_.command.any();
  out_.onset = out_.alarm && !previous;
  if (out_.alarm) {
    out_.latched = config_.latch;
  }
  return out_;
}

}  // namespace ids_core