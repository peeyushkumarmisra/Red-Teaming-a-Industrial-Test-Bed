// =============================================================================
//  replay_main.cpp - the ids_replay command (C21)
// -----------------------------------------------------------------------------
//  ids_replay --urdf ROBOT.urdf --params ids.yaml --reference REF.npy
//             --measurement MEAS.npy --output OUT.npy
//             (--calibrate CAL.yaml [--duration S] | --calibration CAL.yaml)
//             [--warmup S] [--set name=value ...]
//  Calibrate: replay a nominal recording and save a calibration file. Arm: replay any
//  recording with a calibration and write the IDS decision for every window. ids_evaluation
//  calls it for the replay baselines; it can also be run by hand.
// =============================================================================

#include <cstdio>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "ids_core/calibration.hpp"
#include "ids_core/replay.hpp"
#include "ids_ros/parameter_file.hpp"
#include "ids_ros/parameters.hpp"

namespace {

const char kUsage[] =
    "usage: ids_replay --urdf FILE --params FILE --reference FILE.npy --measurement FILE.npy\n"
    "                  --output FILE.npy (--calibrate OUT.yaml [--duration S] |\n"
    "                  --calibration IN.yaml) [--warmup S] [--set name=value ...]\n";

std::string readText(const std::string& path) {
  std::ifstream f(path);
  if (!f) {
    throw std::invalid_argument("cannot open " + path);
  }
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

int run(int argc, char** argv) {
  std::map<std::string, std::string> opt;
  std::vector<std::string> sets;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (key == "--help" || key == "-h") {
      std::fputs(kUsage, stdout);
      return 0;
    }
    if (key.rfind("--", 0) != 0 || i + 1 >= argc) {
      std::fputs(kUsage, stderr);
      return 2;
    }
    if (key == "--set") {
      sets.emplace_back(argv[++i]);
    } else {
      opt[key.substr(2)] = argv[++i];
    }
  }
  for (const char* required : {"urdf", "params", "reference", "measurement", "output"}) {
    if (!opt.count(required)) {
      std::fprintf(stderr, "missing --%s\n%s", required, kUsage);
      return 2;
    }
  }
  // Exactly one mode: make a calibration or use one.
  if (opt.count("calibrate") == opt.count("calibration")) {
    std::fprintf(stderr, "give exactly one of --calibrate and --calibration\n%s", kUsage);
    return 2;
  }
  // Same parameter file as the node, then the --set overrides.
  ids_ros::ParameterMap params = ids_ros::parametersFromFile(opt["params"]);
  for (const auto& s : sets) {
    const auto [name, value] = ids_ros::parseOverride(s);
    params[name] = value;
  }
  const ids_core::IdsConfig config = ids_ros::configFromParameters(params);
  const std::string urdf = readText(opt["urdf"]);
  const ids_core::Array2 references = ids_core::readNpy(opt["reference"]);
  const ids_core::Array2 measurements = ids_core::readNpy(opt["measurement"]);
  ids_core::ReplayOptions options;
  if (opt.count("warmup")) {
    options.warmup = std::stod(opt["warmup"]);
  }
  ids_core::Calibration calibration;
  if (opt.count("calibration")) {
    calibration = ids_core::loadCalibration(opt["calibration"]);
    options.calibration = &calibration;
  } else if (opt.count("duration")) {
    options.calibration_duration = std::stod(opt["duration"]);
  }
  const ids_core::ReplayResult result =
      ids_core::replay(config, urdf, references, measurements, options);
  ids_core::writeNpy(opt["output"], result.windows);
  std::fprintf(stderr, "fingerprint %s\n%zu references, %zu measurements, %zu windows\n",
               result.fingerprint.c_str(), references.rows, measurements.rows,
               result.windows.rows);
  if (result.calibrated) {
    ids_core::saveCalibration(result.calibration, opt["calibrate"]);
    std::fprintf(stderr, "calibration saved to %s: %ld samples, limit %.3f, slow limit %.3f\n",
                 opt["calibrate"].c_str(), result.calibration.samples, result.calibration.limit,
                 result.calibration.slow_limit);
  } else {
    std::fprintf(stderr, "alarm onsets %d, alarm windows %ld\n", result.onsets,
                 result.alarm_windows);
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    return run(argc, argv);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "ids_replay: %s\n", e.what());
    return 1;
  }
}
