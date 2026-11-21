// =============================================================================
//  robot_model.hpp - rigid-body model of the iiwa (Pinocchio)
// -----------------------------------------------------------------------------
//  Gravity, forward and inverse dynamics, mass matrix and their derivatives for the
//  7-joint arm, plus the workpiece held in the gripper.
//  Why it matters: the controller needs gravity compensation, and the IDS needs the
//  full dynamics (and their derivatives for the Kalman filter) to predict motion.
//  All buffers are allocated once, so calls are safe inside a real-time loop.
// =============================================================================

#pragma once

#include <array>
#include <string>

#include <Eigen/Core>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include "iiwa_control_law/types.hpp"

namespace iiwa_control_law {

// Workpiece in the gripper: mass, centre of mass and inertia in the flange frame.
struct Payload {
  double mass = 0.0;
  Eigen::Vector3d com = Eigen::Vector3d::Zero();
  Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();
};

// The 5 kg, 0.1 m cube of the experiments, held with its centre 0.204 m along the link_7 axis.
// The simulation spawns exactly this cube (iiwa_sim / ids_experiments payload).
inline Payload defaultPayload() {
  Payload p;
  p.mass = 5.0;
  p.com = Eigen::Vector3d(0.0, 0.0, 0.204);
  p.inertia = Eigen::Matrix3d::Identity() * 0.008333;
  return p;
}

// Thin wrapper around a Pinocchio model of the 7-joint arm.
class RobotModel {
 public:
  static RobotModel fromUrdfString(const std::string& urdf, const std::string& tip_joint = "joint_a7");
  static RobotModel fromUrdfFile(const std::string& path, const std::string& tip_joint = "joint_a7");

  // Add (or remove) the workpiece to the last link's inertia.
  void setPayload(const Payload& payload);
  const Payload& payload() const { return payload_; }

  Vec7 gravity(const Vec7& q);
  void gravityDerivative(const Vec7& q, Mat7& dg_dq);
  Vec7 forwardDynamics(const Vec7& q, const Vec7& v, const Vec7& tau);
  // Derivatives used to linearise the closed loop for the Kalman filter.
  void forwardDynamicsDerivatives(const Vec7& q, const Vec7& v, const Vec7& tau, Mat7& da_dq,
                                  Mat7& da_dv, Mat7& minv);
  Vec7 inverseDynamics(const Vec7& q, const Vec7& v, const Vec7& a);
  Mat7 massMatrix(const Vec7& q);

  const std::array<std::string, kDof>& jointNames() const { return joint_names_; }
  Vec7 effortLimits() const { return effort_limits_; }
  Vec7 lowerLimits() const { return lower_limits_; }
  Vec7 upperLimits() const { return upper_limits_; }
  const pinocchio::Model& model() const { return model_; }

 private:
  RobotModel(pinocchio::Model model, const std::string& tip_joint);

  pinocchio::Model model_;
  pinocchio::Data data_;
  pinocchio::JointIndex tip_;
  pinocchio::Inertia tip_inertia_;
  Payload payload_;
  std::array<std::string, kDof> joint_names_;
  Vec7 effort_limits_;
  Vec7 lower_limits_;
  Vec7 upper_limits_;
  // Work buffers, allocated once (no heap allocation per call).
  Eigen::VectorXd tau_buffer_;
  Eigen::MatrixXd partial_dq_;
  Eigen::MatrixXd partial_dv_;
  Eigen::MatrixXd partial_dtau_;
};

}  // namespace iiwa_control_law
