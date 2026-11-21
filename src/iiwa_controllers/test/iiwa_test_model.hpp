#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

#include "iiwa_control_law/control_law.hpp"
#include "iiwa_control_law/robot_model.hpp"

namespace iiwa_controllers::test {

inline std::string iiwaUrdf() {
  struct Link {
    double mass;
    std::array<double, 3> com;
    std::array<double, 3> inertia;
  };
  struct Joint {
    std::array<double, 3> xyz;
    std::array<int, 3> axis;
    double effort;
    double limit;
  };
  const std::array<Link, 7> links = {{
      {4.0, {0.0, -0.03, 0.12}, {0.1, 0.09, 0.02}},
      {4.0, {0.0003, 0.059, 0.042}, {0.05, 0.018, 0.044}},
      {3.0, {0.0, 0.03, 0.13}, {0.08, 0.075, 0.01}},
      {2.7, {0.0, 0.067, 0.034}, {0.03, 0.01, 0.029}},
      {1.7, {0.0001, 0.021, 0.076}, {0.02, 0.018, 0.005}},
      {1.8, {0.0, 0.0006, 0.0004}, {0.025, 0.0136, 0.0247}},
      {0.42857142857142855, {0.0, 0.0, 0.02}, {0.01, 0.01, 0.01}},
  }};
  const std::array<Joint, 7> joints = {{
      {{0.0, 0.0, 0.0}, {0, 0, 1}, 320.0, 2.9668},
      {{-0.00043624, 0.0, 0.36}, {0, 1, 0}, 320.0, 2.0942},
      {{0.0, 0.0, 0.0}, {0, 0, 1}, 176.0, 2.9668},
      {{0.00043624, 0.0, 0.42}, {0, -1, 0}, 176.0, 2.0942},
      {{0.0, 0.0, 0.0}, {0, 0, 1}, 110.0, 2.9668},
      {{0.0, 0.0, 0.4}, {0, 1, 0}, 40.0, 2.0942},
      {{0.0, 0.0, 0.0}, {0, 0, 1}, 40.0, 3.0541},
  }};
  std::ostringstream u;
  u << std::setprecision(17);
  u << "<?xml version=\"1.0\"?><robot name=\"iiwa_test\"><link name=\"world\"/>"
    << "<link name=\"link_0\"/><joint name=\"base\" type=\"fixed\"><parent link=\"world\"/>"
    << "<child link=\"link_0\"/></joint>";
  for (std::size_t i = 0; i < 7; ++i) {
    const Link& l = links[i];
    const Joint& j = joints[i];
    u << "<link name=\"link_" << i + 1 << "\"><inertial><origin xyz=\"" << l.com[0] << ' '
      << l.com[1] << ' ' << l.com[2] << "\" rpy=\"0 0 0\"/><mass value=\"" << l.mass
      << "\"/><inertia ixx=\"" << l.inertia[0] << "\" ixy=\"0\" ixz=\"0\" iyy=\"" << l.inertia[1]
      << "\" iyz=\"0\" izz=\"" << l.inertia[2] << "\"/></inertial></link>";
    u << "<joint name=\"joint_a" << i + 1 << "\" type=\"revolute\"><origin xyz=\"" << j.xyz[0]
      << ' ' << j.xyz[1] << ' ' << j.xyz[2] << "\" rpy=\"0 0 0\"/><parent link=\"link_" << i
      << "\"/><child link=\"link_" << i + 1 << "\"/><axis xyz=\"" << j.axis[0] << ' '
      << j.axis[1] << ' ' << j.axis[2] << "\"/><limit effort=\"" << j.effort << "\" lower=\""
      << -j.limit << "\" upper=\"" << j.limit << "\" velocity=\"2.0\"/></joint>";
  }
  u << "</robot>";
  return u.str();
}

inline iiwa_control_law::RobotModel iiwaModel() {
  return iiwa_control_law::RobotModel::fromUrdfString(iiwaUrdf());
}

inline iiwa_control_law::Vec7 homePose() {
  iiwa_control_law::Vec7 q;
  q << 0.0, 0.5, 0.0, -1.2, 0.0, 0.8, 0.0;
  return q;
}

inline iiwa_control_law::ReferenceSample hold(double t, std::uint64_t seq,
                                              const iiwa_control_law::Vec7& position) {
  iiwa_control_law::ReferenceSample s;
  s.stamp = t;
  s.seq = seq;
  s.position = position;
  return s;
}

inline iiwa_control_law::ReferenceSample sinusoid(double t, std::uint64_t seq) {
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

}  // namespace iiwa_controllers::test