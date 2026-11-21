// =============================================================================
//  tracking_core.hpp - controller logic without ROS
// -----------------------------------------------------------------------------
//  States: waiting (hold) -> resuming (blend in) -> tracking; halted on an IDS alarm
//  (only if enabled) or starved when references stop; both hold the measured position
//  and resume with a blend.
//  Also here: the decimator that turns 10 controller ticks into one IDS window, and
//  the tick clock that gives exact 1 ms steps in simulation (B19).
// =============================================================================

#pragma once

#include <cstdint>

#include "iiwa_control_law/control_law.hpp"
#include "iiwa_control_law/robot_model.hpp"

namespace iiwa_controllers {

using iiwa_control_law::Vec7;

// Gains and payload must match the IDS's settings (same defaults).
struct TrackingConfig {
  iiwa_control_law::Gains gains = iiwa_control_law::Gains::iiwaDefault();
  iiwa_control_law::Payload nominal_payload = iiwa_control_law::defaultPayload();
  double reference_hold = 0.05;
  // No new reference for 0.5 s: stop and hold (starved).
  double reference_timeout = 0.5;
  // Blend from the hold position back onto the reference over 0.5 s.
  double resume_time = 0.5;
  double nominal_period = 0.001;
  // A period over 2.5 ms counts as a missed deadline.
  double overrun_factor = 2.5;
  // Safety reaction to the IDS. Off in the v1.2 runs (the halt chain is tested in
  // unit tests and planned for simulation in v1.6).
  bool halt_on_alarm = false;
};

// Controller modes; anything other than tracking is reported as halted to the IDS.
enum class TrackingState { kWaiting, kTracking, kHalted, kStarved, kResuming };

struct TrackingOutput {
  Vec7 torque = Vec7::Zero();
  TrackingState state = TrackingState::kWaiting;
  bool halted = true;
  std::uint64_t reference_seq = 0;
};

// Publishes every Nth tick. A window is marked halted if ANY tick in it was not
// tracking, so the IDS never predicts tracking for a window that held or blended (B6).
class MeasurementDecimator {
 public:
  explicit MeasurementDecimator(int every) : every_(every > 0 ? every : 1) {}

  void reset() {
    count_ = 0;
    pending_ = false;
    first_ = true;
  }

  bool tick(bool halted, bool& window_halted) {
    const bool publish = count_ % every_ == 0;
    ++count_;
    if (!publish) {
      pending_ = pending_ || halted;
      return false;
    }
    window_halted = first_ ? halted : (pending_ || halted);
    first_ = false;
    pending_ = halted;
    return true;
  }

  void defer(bool window_halted) { pending_ = pending_ || window_halted; }

 private:
  int every_;
  long count_ = 0;
  bool pending_ = false;
  bool first_ = true;
};

// Simulation clock for the controller: anchor on the first update, then advance
// exactly one period per update. Measurement stamps become exact 10 ms windows that
// the IDS can replay tick for tick (B19).
class TickClock {
 public:
  explicit TickClock(std::int64_t period_ns = 1000000)
      : period_ns_(period_ns > 0 ? period_ns : 1) {}

  void reset() { started_ = false; }

  std::int64_t next(std::int64_t external_ns) {
    if (!started_) {
      epoch_ns_ = external_ns;
      ticks_ = 0;
      started_ = true;
    }
    return epoch_ns_ + period_ns_ * ticks_++;
  }

  std::int64_t period_ns() const { return period_ns_; }

 private:
  std::int64_t period_ns_;
  std::int64_t epoch_ns_ = 0;
  std::int64_t ticks_ = 0;
  bool started_ = false;
};

// The controller's decision logic and torque computation.
class TrackingCore {
 public:
  TrackingCore(iiwa_control_law::RobotModel& model, const TrackingConfig& config);

  void setReference(const iiwa_control_law::ReferenceSample& sample);
  const TrackingOutput& update(double t, double period, const Vec7& q, const Vec7& v, bool alarm);

  const TrackingConfig& config() const { return config_; }
  const TrackingOutput& output() const { return out_; }
  const Vec7& holdPosition() const { return hold_; }
  bool hasReference() const { return has_reference_; }
  long overruns() const { return overruns_; }

 private:
  void applyContext();
  void enterHold(const Vec7& q, TrackingState state);
  void startResume(double t);

  iiwa_control_law::RobotModel& model_;
  TrackingConfig config_;
  iiwa_control_law::ReferenceSample reference_;
  bool has_reference_ = false;
  int context_ = -1;
  double context_mass_ = -1.0;
  Vec7 hold_ = Vec7::Zero();
  bool hold_set_ = false;
  double resume_start_ = 0.0;
  long overruns_ = 0;
  TrackingOutput out_;
};

}  // namespace iiwa_controllers
