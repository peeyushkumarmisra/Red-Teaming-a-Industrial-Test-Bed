#pragma once

#include <string>
#include <vector>

#include "ids_core/config.hpp"
#include "ids_core/types.hpp"

namespace ids_core {

struct Calibration {
  std::string fingerprint;
  double duration_s = 0.0;
  long samples = 0;
  Vec7 physics_mean = Vec7::Zero();
  Vec7 physics_std = Vec7::Ones();
  Vec7 command_mean = Vec7::Zero();
  Vec7 command_std = Vec7::Ones();
  Vec7 physics_scale = Vec7::Ones();
  Vec7 slow_scale = Vec7::Ones();
  Vec7 command_scale = Vec7::Ones();
  Vec7 friction_initial = Vec7::Zero();
  double limit = 4.0;
  double slow_limit = 4.0;
};

std::string modelSignature(const std::string& urdf, const std::string& tip_joint);
std::string configSignature(const IdsConfig& config);
std::string computeFingerprint(const std::string& urdf, const IdsConfig& config);

void saveCalibration(const Calibration& calibration, const std::string& path);
Calibration loadCalibration(const std::string& path);

class CalibrationRecorder {
 public:
  void clear();
  void add(const Vec7& physics, const Vec7* command);
  long samples() const { return static_cast<long>(physics_.size()); }
  Calibration finalize(const IdsConfig& config, const std::string& fingerprint,
                       const Vec7& friction, double duration_s) const;

 private:
  std::vector<Vec7> physics_;
  std::vector<Vec7> command_;
};

}  // namespace ids_core