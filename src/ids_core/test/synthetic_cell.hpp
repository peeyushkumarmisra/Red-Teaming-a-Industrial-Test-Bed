// =============================================================================
//  synthetic_cell.hpp - a simulated robot cell without Gazebo
// -----------------------------------------------------------------------------
//  A Pinocchio plant at 4 kHz, the real control law at 1 kHz, encoder noise, friction
//  that wears, and optional attacks (setpoint step, setpoint ramp, torque bias). It
//  feeds the real IdsPipeline, so detection, attribution and false alarms can be tested
//  in seconds on any machine. Limitation: it uses the same URDF as the IDS, so it has
//  no model mismatch; Gazebo runs test that.
// =============================================================================

#pragma once

#include <cmath>
#include <cstdint>
#include <numbers>
#include <random>
#include <vector>

#include "ids_core/ids_pipeline.hpp"
#include "iiwa_control_law/control_law.hpp"
#include "iiwa_control_law/robot_model.hpp"
#include "test_helpers.hpp"

namespace ids_core::test {

// Attacks the cell can inject.
enum class AttackKind { kNone, kSetpointStep, kSetpointRamp, kTorqueBias };

struct Attack {
  AttackKind kind = AttackKind::kNone;
  int joint = 0;
  double magnitude = 0.0;
  double start = 0.0;
};

// Plant step, controller rate, noise, friction and wear settings.
struct CellConfig {
  double plant_dt = 0.00025;
  double controller_dt = 0.001;
  double ids_dt = 0.01;
  double reference_dt = 0.01;
  double reference_hold = 0.05;
  double position_resolution = 2.0 * std::numbers::pi / 1048576.0;
  double velocity_noise = 1e-3;
  Vec7 friction = (Vec7() << 4.0, 4.0, 2.0, 2.0, 0.4, 0.2, 0.1).finished();
  double friction_growth_per_s = 0.0;
  double payload_mass = 5.0;
  double context_on = -1.0;
  double context_off = -1.0;
  double attach_at = -1.0;
  double detach_at = -1.0;
  std::uint64_t seed = 1;
};

// Runs plant, controller and IDS together; records alarms and attribution.
class SyntheticCell {
 public:
  explicit SyntheticCell(const CellConfig& config = CellConfig{})
      : config_(config),
        plant_(iiwa_control_law::RobotModel::fromUrdfFile(urdfPath())),
        controller_(iiwa_control_law::RobotModel::fromUrdfFile(urdfPath())),
        gains_(iiwa_control_law::Gains::iiwaDefault()),
        rng_(config.seed),
        noise_(0.0, 1.0) {
    const auto s = sinusoidSample(0.0, 0);
    q_ = s.position;
    v_ = s.velocity;
    friction_ = config_.friction;
    plant_substeps_ = static_cast<int>(std::lround(config_.controller_dt / config_.plant_dt));
    ticks_per_sample_ = static_cast<int>(std::lround(config_.ids_dt / config_.controller_dt));
    ticks_per_reference_ =
        static_cast<int>(std::lround(config_.reference_dt / config_.controller_dt));
  }

  void setAttack(const Attack& attack) { attack_ = attack; }
  // Optionally keep every trusted reference the cell sends, so a test can save the run as
  // arrays and replay it offline (test_replay).
  void logReferences(std::vector<iiwa_control_law::ReferenceSample>* log) { reference_log_ = log; }
  double time() const { return static_cast<double>(tick_) * config_.controller_dt; }
  const Vec7& friction() const { return friction_; }
  const Vec7& position() const { return q_; }

  Measurement step(IdsPipeline& ids) {
    Measurement m;
    for (int c = 0; c < ticks_per_sample_; ++c) {
      const double t = time();
      if (tick_ % ticks_per_reference_ == 0) {
        auto trusted = sinusoidSample(t, seq_++);
        if (within(config_.context_on, config_.context_off, t)) {
          trusted.payload_context = 1;
          trusted.payload_mass = config_.payload_mass;
        }
        ids.addReference(trusted);
        if (reference_log_ != nullptr) {
          reference_log_->push_back(trusted);
        }
        received_ = intercept(trusted);
      }
      carry(controller_, controller_payload_, received_.payload_context == 1);
      carry(plant_, plant_payload_, within(config_.attach_at, config_.detach_at, t));
      Vec7 qm;
      Vec7 vm;
      sense(qm, vm);
      const auto r = iiwa_control_law::referenceAt(received_, t, config_.reference_hold);
      Vec7 tau = iiwa_control_law::computeTorque(gains_, qm, vm, r, controller_.gravity(qm));
      if (active(AttackKind::kTorqueBias, t)) {
        tau(attack_.joint) += attack_.magnitude;
      }
      m.stamp = t;
      m.q = qm;
      m.v = vm;
      m.tau = tau;
      m.reference_seq = received_.seq;
      m.halted = false;
      const double h = config_.plant_dt;
      for (int p = 0; p < plant_substeps_; ++p) {
        const Vec7 a = plant_.forwardDynamics(q_, v_, tau - friction_.cwiseProduct(v_));
        v_ += h * a;
        q_ += h * v_;
      }
      friction_ *= 1.0 + config_.friction_growth_per_s * config_.controller_dt;
      ++tick_;
    }
    return m;
  }

 private:
  static bool within(double from, double to, double t) {
    return from >= 0.0 && t >= from - 1e-9 && (to < 0.0 || t < to - 1e-9);
  }

  void carry(iiwa_control_law::RobotModel& model, bool& carrying, bool wanted) {
    if (wanted == carrying) return;
    iiwa_control_law::Payload p = iiwa_control_law::defaultPayload();
    p.mass = wanted ? config_.payload_mass : 0.0;
    model.setPayload(p);
    carrying = wanted;
  }

  bool active(AttackKind kind, double t) const {
    return attack_.kind == kind && t >= attack_.start - 1e-9;
  }

  iiwa_control_law::ReferenceSample intercept(const iiwa_control_law::ReferenceSample& s) const {
    iiwa_control_law::ReferenceSample out = s;
    if (active(AttackKind::kSetpointStep, s.stamp)) {
      out.position(attack_.joint) += attack_.magnitude;
    }
    if (active(AttackKind::kSetpointRamp, s.stamp)) {
      out.position(attack_.joint) += attack_.magnitude * (s.stamp - attack_.start);
      out.velocity(attack_.joint) += attack_.magnitude;
    }
    return out;
  }

  void sense(Vec7& qm, Vec7& vm) {
    const double res = config_.position_resolution;
    for (int i = 0; i < kDof; ++i) {
      qm(i) = res > 0.0 ? std::round(q_(i) / res) * res : q_(i);
      vm(i) = v_(i) + config_.velocity_noise * noise_(rng_);
    }
  }

  CellConfig config_;
  iiwa_control_law::RobotModel plant_;
  iiwa_control_law::RobotModel controller_;
  iiwa_control_law::Gains gains_;
  std::mt19937_64 rng_;
  std::normal_distribution<double> noise_;
  Attack attack_;
  std::vector<iiwa_control_law::ReferenceSample>* reference_log_ = nullptr;
  iiwa_control_law::ReferenceSample received_;
  Vec7 q_ = Vec7::Zero();
  Vec7 v_ = Vec7::Zero();
  Vec7 friction_ = Vec7::Zero();
  std::uint64_t seq_ = 0;
  long tick_ = 0;
  bool controller_payload_ = false;
  bool plant_payload_ = false;
  int plant_substeps_ = 4;
  int ticks_per_sample_ = 10;
  int ticks_per_reference_ = 10;
};

}  // namespace ids_core::test
