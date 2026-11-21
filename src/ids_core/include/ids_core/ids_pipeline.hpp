#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "ids_core/calibration.hpp"
#include "ids_core/closed_loop_model.hpp"
#include "ids_core/config.hpp"
#include "ids_core/detector.hpp"
#include "ids_core/dual_ewma.hpp"
#include "ids_core/ekf.hpp"
#include "ids_core/reference_track.hpp"
#include "iiwa_control_law/robot_model.hpp"

namespace ids_core {

enum class Mode { kWaiting = 0, kCalibrating = 1, kArmed = 2, kFault = 3 };

struct Measurement {
  double stamp = 0.0;
  Vec7 q = Vec7::Zero();
  Vec7 v = Vec7::Zero();
  Vec7 tau = Vec7::Zero();
  std::uint64_t reference_seq = 0;
  bool halted = false;
};

struct IdsOutput {
  Mode mode = Mode::kWaiting;
  bool valid = false;
  bool alarm = false;
  bool latched = false;
  int primary_joint = -1;
  Mask7 physics_alarm = Mask7::Constant(false);
  Mask7 command_alarm = Mask7::Constant(false);
  Vec7 disturbance = Vec7::Zero();
  Vec7 command_error = Vec7::Zero();
  Vec7 physics_statistic = Vec7::Zero();
  Vec7 physics_slow_statistic = Vec7::Zero();
  Vec7 command_statistic = Vec7::Zero();
  Vec7 friction = Vec7::Zero();
  double limit = 0.0;
  double nis = 0.0;
};

class IdsPipeline {
 public:
  IdsPipeline(const IdsConfig& config, const std::string& urdf);

  const std::string& fingerprint() const { return fingerprint_; }
  const IdsConfig& config() const { return config_; }
  Mode mode() const { return mode_; }
  int detectorWarmupWindows() const { return detector_warmup_; }

  void startCalibration();
  Calibration finishCalibration();
  void arm(const Calibration& calibration);
  void resetAlarm();

  void addReference(const iiwa_control_law::ReferenceSample& sample);
  const IdsOutput& process(const Measurement& m);

  const Ekf& ekf() const { return ekf_; }
  const Calibration& calibration() const { return calibration_; }

 private:
  void applyContext(double t);

  IdsConfig config_;
  std::string fingerprint_;
  iiwa_control_law::RobotModel robot_;
  ClosedLoopModel model_;
  Ekf ekf_;
  ReferenceTrack track_;
  DualEwma physics_ewma_;
  DualEwma command_ewma_;
  Detector detector_;
  CalibrationRecorder recorder_;
  Calibration calibration_;
  IdsOutput out_;
  Mode mode_ = Mode::kWaiting;
  bool initialized_ = false;
  double last_stamp_ = 0.0;
  double calibration_start_ = -1.0;
  Vec7 last_tau_ = Vec7::Zero();
  int context_ = -1;
  double context_mass_ = 0.0;
  double context_change_ = -1e9;
  bool use_slow_ = false;
  bool use_command_ = false;
  int primary_joint_ = -1;
  int detector_windows_ = 0;
  int detector_warmup_ = 0;
};

}  // namespace ids_core