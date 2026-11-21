#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <vector>

#include "synthetic_cell.hpp"

int main(int argc, char** argv) {
  const int samples = argc > 1 ? std::atoi(argv[1]) : 5000;
  std::ifstream f(ids_core::test::urdfPath());
  std::stringstream ss;
  ss << f.rdbuf();
  ids_core::IdsConfig cfg;
  ids_core::IdsPipeline ids(cfg, ss.str());
  ids_core::Calibration cal;
  cal.fingerprint = ids.fingerprint();
  cal.physics_std = ids_core::Vec7::Constant(0.3);
  ids.arm(cal);
  ids_core::test::SyntheticCell cell;
  for (int k = 0; k < 200; ++k) ids.process(cell.step(ids));
  std::vector<double> us;
  us.reserve(static_cast<std::size_t>(samples));
  for (int k = 0; k < samples; ++k) {
    const ids_core::Measurement m = cell.step(ids);
    const auto t0 = std::chrono::steady_clock::now();
    ids.process(m);
    const auto t1 = std::chrono::steady_clock::now();
    us.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
  }
  std::sort(us.begin(), us.end());
  const auto pick = [&](double q) {
    return us[static_cast<std::size_t>(q * static_cast<double>(us.size() - 1))];
  };
  std::printf("ids_process_us samples=%d median=%.1f p90=%.1f p99=%.1f max=%.1f\n", samples,
              pick(0.5), pick(0.9), pick(0.99), us.back());
  return 0;
}