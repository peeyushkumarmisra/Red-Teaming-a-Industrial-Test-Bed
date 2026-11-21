// =============================================================================
//  tracking_core.cpp - state machine and torque
// -----------------------------------------------------------------------------
//  Every tick: apply the trusted payload context (gravity of the held part), update the
//  state machine, then compute the torque with the shared control law
//  (iiwa_control_law), the same law the IDS replays.
// =============================================================================

#include "iiwa_controllers/tracking_core.hpp"

#include <algorithm>

namespace iiwa_controllers {

using iiwa_control_law::computeTorque;
using iiwa_control_law::holdTorque;
using iiwa_control_law::ReferencePoint;
using iiwa_control_law::referenceAt;

TrackingCore::TrackingCore(iiwa_control_law::RobotModel& model, const TrackingConfig& config)
    : model_(model), config_(config) {}

void TrackingCore::setReference(const iiwa_control_law::ReferenceSample& sample) {
  reference_ = sample;
  has_reference_ = true;
}

// Gravity feed-forward follows the payload context in the reference.
void TrackingCore::applyContext() {
  if (!has_reference_) {
    return;
  }
  const int context = reference_.payload_context;
  if (context == context_ && reference_.payload_mass == context_mass_) {
    return;
  }
  iiwa_control_law::Payload payload = config_.nominal_payload;
  payload.mass = context == 0 ? 0.0 : reference_.payload_mass;
  model_.setPayload(payload);
  context_ = context;
  context_mass_ = reference_.payload_mass;
}

void TrackingCore::enterHold(const Vec7& q, TrackingState state) {
  hold_ = q;
  out_.state = state;
}

void TrackingCore::startResume(double t) {
  resume_start_ = t;
  out_.state = config_.resume_time > 0.0 ? TrackingState::kResuming : TrackingState::kTracking;
}

const TrackingOutput& TrackingCore::update(double t, double period, const Vec7& q, const Vec7& v,
                                           bool alarm) {
  // Deadline guard: count late updates (the IDS assumes 1 ms ticks).
  if (period > config_.overrun_factor * config_.nominal_period) {
    ++overruns_;
  }
  if (!hold_set_) {
    hold_ = q;
    hold_set_ = true;
  }
  applyContext();
  // Starved: the reference stream stopped.
  const bool stale = has_reference_ && t - reference_.stamp > config_.reference_timeout;
  // Halt only if the halt chain is enabled.
  const bool halt = config_.halt_on_alarm && alarm;
  switch (out_.state) {
    case TrackingState::kWaiting:
      if (has_reference_ && !stale && !halt) startResume(t);
      break;
    case TrackingState::kTracking:
      if (halt) {
        enterHold(q, TrackingState::kHalted);
      } else if (stale) {
        enterHold(q, TrackingState::kStarved);
      }
      break;
    case TrackingState::kHalted:
      if (!halt) {
        if (stale) {
          out_.state = TrackingState::kStarved;
        } else {
          startResume(t);
        }
      }
      break;
    case TrackingState::kStarved:
      if (halt) {
        out_.state = TrackingState::kHalted;
      } else if (!stale) {
        startResume(t);
      }
      break;
    case TrackingState::kResuming:
      if (halt) {
        enterHold(q, TrackingState::kHalted);
      } else if (stale) {
        enterHold(q, TrackingState::kStarved);
      } else if (t - resume_start_ >= config_.resume_time) {
        out_.state = TrackingState::kTracking;
      }
      break;
  }
  const Vec7 g = model_.gravity(q);
  switch (out_.state) {
    case TrackingState::kTracking:
      out_.torque = computeTorque(config_.gains, q, v,
                                  referenceAt(reference_, t, config_.reference_hold), g);
      break;
    // Blend: move the target from the hold position to the reference over resume_time.
    case TrackingState::kResuming: {
      const double alpha = std::clamp((t - resume_start_) / config_.resume_time, 0.0, 1.0);
      const ReferencePoint r = referenceAt(reference_, t, config_.reference_hold);
      const ReferencePoint blended{hold_ + alpha * (r.position - hold_), alpha * r.velocity};
      out_.torque = computeTorque(config_.gains, q, v, blended, g);
      break;
    }
    // Waiting, halted or starved: hold the position where the arm was.
    default:
      out_.torque = holdTorque(config_.gains, q, v, hold_, g);
      break;
  }
  // Tell the IDS whenever the controller is not simply tracking.
  out_.halted = out_.state != TrackingState::kTracking;
  out_.reference_seq = has_reference_ ? reference_.seq : 0;
  return out_;
}

}  // namespace iiwa_controllers
