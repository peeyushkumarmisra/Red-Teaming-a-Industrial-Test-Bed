// =============================================================================
//  replay.cpp - offline replay of recorded runs (C21)
// -----------------------------------------------------------------------------
//  Mirrors ids_node: every trusted reference whose stamp is not later than the
//  measurement is added first, measurements before the first reference plus the warm-up
//  are skipped, and each remaining measurement is one 10 ms window. Model, filter,
//  statistics, limits and decisions are the same code as live, so a difference between
//  live and replay can only come from message timing in the live system (a reference that
//  arrived after its measurement).
// =============================================================================

#include "ids_core/replay.hpp"

#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace ids_core {

namespace {

const char kMagic[] = "\x93NUMPY";

std::size_t parseShapeValue(const std::string& header, std::size_t& pos) {
  while (pos < header.size() && (header[pos] == ' ' || header[pos] == ',')) {
    ++pos;
  }
  std::size_t end = pos;
  while (end < header.size() && header[end] >= '0' && header[end] <= '9') {
    ++end;
  }
  if (end == pos) {
    throw std::runtime_error("npy: cannot read the array shape");
  }
  const std::size_t value = std::stoull(header.substr(pos, end - pos));
  pos = end;
  return value;
}

Vec7 at7(const double* p) { return Eigen::Map<const Vec7>(p); }

void check(const Array2& a, std::size_t cols, const char* what) {
  if (a.cols != cols) {
    throw std::runtime_error(std::string(what) + " must have " + std::to_string(cols) +
                             " columns, found " + std::to_string(a.cols));
  }
}

}  // namespace

// Parse the .npy header (magic string, version, then a dict with descr, fortran_order and
// shape) and read the raw doubles.
Array2 readNpy(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) {
    throw std::runtime_error("cannot open " + path);
  }
  char magic[6];
  f.read(magic, 6);
  if (!f || std::memcmp(magic, kMagic, 6) != 0) {
    throw std::runtime_error(path + ": not a .npy file");
  }
  unsigned char version[2];
  f.read(reinterpret_cast<char*>(version), 2);
  std::uint32_t header_len = 0;
  if (version[0] == 1) {
    unsigned char b[2];
    f.read(reinterpret_cast<char*>(b), 2);
    header_len = b[0] | (b[1] << 8);
  } else {
    unsigned char b[4];
    f.read(reinterpret_cast<char*>(b), 4);
    header_len = b[0] | (b[1] << 8) | (b[2] << 16) | (static_cast<std::uint32_t>(b[3]) << 24);
  }
  std::string header(header_len, ' ');
  f.read(header.data(), header_len);
  if (!f) {
    throw std::runtime_error(path + ": truncated header");
  }
  if (header.find("'<f8'") == std::string::npos) {
    throw std::runtime_error(path + ": only little-endian float64 arrays are supported");
  }
  if (header.find("'fortran_order': False") == std::string::npos) {
    throw std::runtime_error(path + ": only C-ordered arrays are supported");
  }
  const std::size_t open = header.find("'shape': (");
  if (open == std::string::npos) {
    throw std::runtime_error(path + ": no shape in header");
  }
  std::size_t pos = open + 10;
  Array2 a;
  a.rows = parseShapeValue(header, pos);
  while (pos < header.size() && header[pos] == ' ') {
    ++pos;
  }
  if (pos >= header.size() || header[pos] != ',') {
    throw std::runtime_error(path + ": expected a 2-D array");
  }
  ++pos;
  a.cols = parseShapeValue(header, pos);
  a.data.resize(a.rows * a.cols);
  f.read(reinterpret_cast<char*>(a.data.data()),
         static_cast<std::streamsize>(a.data.size() * sizeof(double)));
  if (!f) {
    throw std::runtime_error(path + ": truncated data");
  }
  return a;
}

// Write version 1.0 .npy files that numpy.load reads directly.
void writeNpy(const std::string& path, const Array2& array) {
  std::ostringstream h;
  h << "{'descr': '<f8', 'fortran_order': False, 'shape': (" << array.rows << ", " << array.cols
    << "), }";
  std::string header = h.str();
  const std::size_t total = 10 + header.size() + 1;
  header.append((64 - total % 64) % 64, ' ');
  header.push_back('\n');
  std::ofstream f(path, std::ios::binary);
  if (!f) {
    throw std::runtime_error("cannot write " + path);
  }
  f.write(kMagic, 6);
  const char version[2] = {1, 0};
  f.write(version, 2);
  const std::uint16_t len = static_cast<std::uint16_t>(header.size());
  const char len_bytes[2] = {static_cast<char>(len & 0xff), static_cast<char>(len >> 8)};
  f.write(len_bytes, 2);
  f.write(header.data(), static_cast<std::streamsize>(header.size()));
  f.write(reinterpret_cast<const char*>(array.data.data()),
          static_cast<std::streamsize>(array.data.size() * sizeof(double)));
}

void appendReference(Array2& array, const iiwa_control_law::ReferenceSample& s) {
  array.cols = kReferenceColumns;
  array.data.push_back(s.stamp);
  array.data.push_back(static_cast<double>(s.seq));
  array.data.push_back(static_cast<double>(s.payload_context));
  array.data.push_back(s.payload_mass);
  for (const Vec7* v : {&s.position, &s.velocity, &s.acceleration}) {
    array.data.insert(array.data.end(), v->data(), v->data() + kDof);
  }
  ++array.rows;
}

void appendMeasurement(Array2& array, const Measurement& m) {
  array.cols = kMeasurementColumns;
  array.data.push_back(m.stamp);
  array.data.push_back(static_cast<double>(m.reference_seq));
  array.data.push_back(m.halted ? 1.0 : 0.0);
  for (const Vec7* v : {&m.q, &m.v, &m.tau}) {
    array.data.insert(array.data.end(), v->data(), v->data() + kDof);
  }
  ++array.rows;
}

ReplayResult replay(const IdsConfig& config, const std::string& urdf, const Array2& references,
                    const Array2& measurements, const ReplayOptions& options) {
  check(references, kReferenceColumns, "references");
  check(measurements, kMeasurementColumns, "measurements");
  IdsPipeline ids(config, urdf);
  ReplayResult result;
  result.fingerprint = ids.fingerprint();
  const bool calibrating = options.calibration == nullptr;
  if (calibrating) {
    ids.startCalibration();
  } else {
    ids.arm(*options.calibration);
  }
  result.windows.cols = kReplayColumns;
  std::size_t r = 0;
  bool have_reference = false;
  double first_reference = 0.0;
  double first_window = -1.0;
  bool previous_alarm = false;
  for (std::size_t i = 0; i < measurements.rows; ++i) {
    const double* m = measurements.row(i);
    const double stamp = m[0];
    // Feed every reference up to this measurement's stamp: the replay assumes references arrive
    // on time.
    while (r < references.rows && references.at(r, 0) <= stamp) {
      const double* s = references.row(r);
      iiwa_control_law::ReferenceSample sample;
      sample.stamp = s[0];
      sample.seq = static_cast<std::uint64_t>(s[1]);
      sample.payload_context = static_cast<std::uint8_t>(s[2]);
      sample.payload_mass = s[3];
      sample.position = at7(s + 4);
      sample.velocity = at7(s + 11);
      sample.acceleration = at7(s + 18);
      ids.addReference(sample);
      if (!have_reference) {
        first_reference = sample.stamp;
        have_reference = true;
      }
      ++r;
    }
    // Same start-up rule as the node: wait for the first reference plus the warm-up.
    if (!have_reference || stamp < first_reference + options.warmup) {
      continue;
    }
    Measurement x;
    x.stamp = stamp;
    x.reference_seq = static_cast<std::uint64_t>(m[1]);
    x.halted = m[2] > 0.5;
    x.q = at7(m + 3);
    x.v = at7(m + 10);
    x.tau = at7(m + 17);
    const IdsOutput& o = ids.process(x);
    if (first_window < 0.0) {
      first_window = stamp;
    }
    const bool onset = o.alarm && !previous_alarm;
    previous_alarm = o.alarm;
    result.alarm_windows += o.alarm ? 1 : 0;
    result.onsets += onset ? 1 : 0;
    auto& d = result.windows.data;
    d.push_back(stamp);
    d.push_back(static_cast<double>(static_cast<int>(o.mode)));
    d.push_back(o.valid ? 1.0 : 0.0);
    d.push_back(o.alarm ? 1.0 : 0.0);
    d.push_back(o.latched ? 1.0 : 0.0);
    d.push_back(static_cast<double>(o.primary_joint));
    d.push_back(onset ? 1.0 : 0.0);
    d.push_back(o.nis);
    for (const Vec7* v : {&o.disturbance, &o.physics_statistic, &o.physics_slow_statistic,
                          &o.command_statistic, &o.friction}) {
      d.insert(d.end(), v->data(), v->data() + kDof);
    }
    for (int j = 0; j < kDof; ++j) {
      d.push_back(o.physics_alarm(j) ? 1.0 : 0.0);
    }
    d.insert(d.end(), o.command_error.data(), o.command_error.data() + kDof);
    ++result.windows.rows;
    // A calibrating replay stops once it has enough windows, like the live calibration mode.
    if (calibrating && stamp - first_window >= options.calibration_duration) {
      result.calibration = ids.finishCalibration();
      result.calibrated = true;
      break;
    }
  }
  if (calibrating && !result.calibrated) {
    throw std::runtime_error("recording ended before the calibration duration was reached");
  }
  return result;
}

}  // namespace ids_core
