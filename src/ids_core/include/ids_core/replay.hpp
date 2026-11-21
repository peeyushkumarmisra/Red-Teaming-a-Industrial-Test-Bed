#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "ids_core/calibration.hpp"
#include "ids_core/config.hpp"
#include "ids_core/ids_pipeline.hpp"

namespace ids_core {

struct Array2 {
  std::size_t rows = 0;
  std::size_t cols = 0;
  std::vector<double> data;

  double at(std::size_t r, std::size_t c) const { return data[r * cols + c]; }
  const double* row(std::size_t r) const { return data.data() + r * cols; }
};

Array2 readNpy(const std::string& path);
void writeNpy(const std::string& path, const Array2& array);

constexpr std::size_t kReferenceColumns = 25;
constexpr std::size_t kMeasurementColumns = 24;
constexpr std::size_t kReplayColumns = 57;

void appendReference(Array2& array, const iiwa_control_law::ReferenceSample& sample);
void appendMeasurement(Array2& array, const Measurement& measurement);

struct ReplayOptions {
  double warmup = 2.0;
  double calibration_duration = 120.0;
  const Calibration* calibration = nullptr;
};

struct ReplayResult {
  Array2 windows;
  bool calibrated = false;
  Calibration calibration;
  std::string fingerprint;
  long alarm_windows = 0;
  int onsets = 0;
};

ReplayResult replay(const IdsConfig& config, const std::string& urdf, const Array2& references,
                    const Array2& measurements, const ReplayOptions& options);

}  // namespace ids_core