// =============================================================================
//  replay.hpp - run the IDS offline on recorded arrays
// -----------------------------------------------------------------------------
//  Reads and writes .npy matrices and feeds recorded trusted references and measurements
//  through the real IdsPipeline, window by window, as the live node does.
//  Why it matters: a recorded run can be scored again with other settings, compared with
//  baseline detectors (ids_evaluation), or used to make a calibration, without ROS,
//  Gazebo or the robot.
//  Inputs:  references   [t, seq, context, mass, q(7), v(7), a(7)]        25 columns
//           measurements [t, reference_seq, halted, q(7), v(7), tau(7)]    24 columns
//           (the layout written by ids_evaluation's export).
//  Output:  one row of 57 numbers per 10 ms window (kReplayColumns).
// =============================================================================

#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "ids_core/calibration.hpp"
#include "ids_core/config.hpp"
#include "ids_core/ids_pipeline.hpp"

namespace ids_core {

// A row-major matrix of doubles: the in-memory form of a 2-D float64 .npy file.
struct Array2 {
  std::size_t rows = 0;
  std::size_t cols = 0;
  std::vector<double> data;

  double at(std::size_t r, std::size_t c) const { return data[r * cols + c]; }
  const double* row(std::size_t r) const { return data.data() + r * cols; }
};

// Only 2-D little-endian float64 arrays in C order are accepted; anything else is an error,
// never a silent conversion.
Array2 readNpy(const std::string& path);
void writeNpy(const std::string& path, const Array2& array);

constexpr std::size_t kReferenceColumns = 25;
constexpr std::size_t kMeasurementColumns = 24;
// Output row: t, mode, valid, alarm, latched, primary joint, onset, NIS, then 7 values each of
//             disturbance, fast statistic, slow statistic, command statistic, friction
//             estimate, physics alarm flags and command error.
constexpr std::size_t kReplayColumns = 57;

void appendReference(Array2& array, const iiwa_control_law::ReferenceSample& sample);
void appendMeasurement(Array2& array, const Measurement& measurement);

// warmup: seconds after the first reference before measurements are used, as in the node.
//         Without a calibration the replay calibrates for calibration_duration seconds and
//         stops; with one it arms and detects.
struct ReplayOptions {
  double warmup = 2.0;
  double calibration_duration = 120.0;
  const Calibration* calibration = nullptr;
};

// The fingerprint (model plus settings) must match a calibration before it can be used to arm,
// exactly as live.
struct ReplayResult {
  Array2 windows;
  bool calibrated = false;
  Calibration calibration;
  std::string fingerprint;
  long alarm_windows = 0;
  int onsets = 0;
};

// Throws if an array has the wrong number of columns or if a calibrating replay ends before its
// duration.
ReplayResult replay(const IdsConfig& config, const std::string& urdf, const Array2& references,
                    const Array2& measurements, const ReplayOptions& options);

}  // namespace ids_core
