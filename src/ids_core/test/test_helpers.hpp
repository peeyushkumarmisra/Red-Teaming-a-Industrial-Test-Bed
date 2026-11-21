#pragma once

#include <cmath>
#include <string>

#include "ids_core/reference_track.hpp"
#include "iiwa_control_law/control_law.hpp"
#include "iiwa_control_law/robot_model.hpp"

namespace ids_core::test {

inline std::string urdfPath() { return IDS_CORE_TEST_URDF; }

inline iiwa_control_law::Vec7 homePose() {
  iiwa_control_law::Vec7 q;
  q << 0.0, 0.5, 0.0, -1.2, 0.0, 0.8, 0.0;
  return q;
}

inline iiwa_control_law::ReferenceSample sinusoidSample(double t, std::uint64_t seq) {
  iiwa_control_law::ReferenceSample s;
  s.stamp = t;
  s.seq = seq;
  const iiwa_control_law::Vec7 q0 = homePose();
  for (int i = 0; i < 7; ++i) {
    const double amp = 0.25 - 0.02 * i;
    const double w = 0.6 + 0.11 * i;
    const double ph = 0.7 * i;
    s.position(i) = q0(i) + amp * std::sin(w * t + ph);
    s.velocity(i) = amp * w * std::cos(w * t + ph);
    s.acceleration(i) = -amp * w * w * std::sin(w * t + ph);
  }
  return s;
}

class Feeder {
 public:
  explicit Feeder(ReferenceTrack& track, double period = 0.01) : track_(track), period_(period) {}
  void until(double t) {
    while (next_ <= t + 1e-9) {
      track_.push(sinusoidSample(next_, seq_++));
      next_ += period_;
    }
  }

 private:
  ReferenceTrack& track_;
  double period_;
  double next_ = 0.0;
  std::uint64_t seq_ = 0;
};

inline void fillTrack(ReferenceTrack& track, double t_end, double period = 0.01) {
  std::uint64_t seq = 0;
  for (double t = 0.0; t <= t_end + 1e-9; t += period) {
    track.push(sinusoidSample(t, seq++));
  }
}

}  // namespace ids_core::test