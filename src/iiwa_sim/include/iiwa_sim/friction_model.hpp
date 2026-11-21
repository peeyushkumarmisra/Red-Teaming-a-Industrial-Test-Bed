// =============================================================================
//  friction_model.hpp - wear schedule and torque composition
// -----------------------------------------------------------------------------
//  FrictionSchedule: the coefficient of each joint over time (see friction.yaml).
//  ForceComposer: adds the friction torque to the torque the controller wrote,
//  without ever adding it twice when the command is not rewritten.
//  Kept free of Gazebo so both are unit-tested.
// =============================================================================

#pragma once

#include <algorithm>
#include <cstddef>
#include <utility>
#include <vector>

namespace iiwa_sim {

struct FrictionChannel {
  double initial = 0.0;
  double rate = 0.0;
  double max = 0.0;
};

class FrictionSchedule {
 public:
  FrictionSchedule() = default;
  FrictionSchedule(double delay, std::vector<FrictionChannel> channels)
      : delay_(delay), channels_(std::move(channels)) {}

  std::size_t size() const { return channels_.size(); }
  double delay() const { return delay_; }
  const FrictionChannel& channel(std::size_t i) const { return channels_[i]; }

  // Linear growth after the delay, capped; never below the initial value.
  double coefficient(std::size_t i, double t) const {
    const FrictionChannel& c = channels_[i];
    const double grown = c.initial + c.rate * std::max(0.0, t - delay_);
    return std::max(0.0, std::min(grown, std::max(c.max, c.initial)));
  }

 private:
  double delay_ = 0.0;
  std::vector<FrictionChannel> channels_;
};

// If the controller rewrote the command this step, add friction to it; if the command
// is our own previous value, replace our old friction term instead of stacking it.
class ForceComposer {
 public:
  double compose(double current_command, double friction_torque) {
    const double base = written_ && current_command == last_written_
                            ? current_command - last_friction_
                            : current_command;
    last_friction_ = friction_torque;
    last_written_ = base + friction_torque;
    written_ = true;
    return last_written_;
  }

 private:
  bool written_ = false;
  double last_written_ = 0.0;
  double last_friction_ = 0.0;
};

}  // namespace iiwa_sim
