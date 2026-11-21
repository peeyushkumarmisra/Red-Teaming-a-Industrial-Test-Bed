// =============================================================================
//  robot_model.cpp - Pinocchio calls behind RobotModel
// -----------------------------------------------------------------------------
//  Loads the arm from URDF, checks it really is a 7-joint serial arm, and wraps the
//  Pinocchio algorithms (RNEA, ABA, CRBA and their derivatives).
// =============================================================================

#include "iiwa_control_law/robot_model.hpp"

#include <stdexcept>
#include <utility>

#include <pinocchio/algorithm/aba-derivatives.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace iiwa_control_law {

RobotModel RobotModel::fromUrdfString(const std::string& urdf, const std::string& tip_joint) {
  pinocchio::Model model;
  pinocchio::urdf::buildModelFromXML(urdf, model);
  return RobotModel(std::move(model), tip_joint);
}

RobotModel RobotModel::fromUrdfFile(const std::string& path, const std::string& tip_joint) {
  pinocchio::Model model;
  pinocchio::urdf::buildModel(path, model);
  return RobotModel(std::move(model), tip_joint);
}

RobotModel::RobotModel(pinocchio::Model model, const std::string& tip_joint)
    : model_(std::move(model)), data_(model_) {
  // Refuse anything that is not the 7-joint arm the IDS was designed for.
  if (model_.nq != kDof || model_.nv != kDof) {
    throw std::runtime_error("RobotModel expects a 7-DoF serial arm");
  }
  if (!model_.existJointName(tip_joint)) {
    throw std::runtime_error("RobotModel tip joint not found: " + tip_joint);
  }
  tip_ = model_.getJointId(tip_joint);
  tip_inertia_ = model_.inertias[tip_];
  for (int i = 0; i < kDof; ++i) {
    joint_names_[i] = model_.names[i + 1];
  }
  effort_limits_ = model_.effortLimit;
  lower_limits_ = model_.lowerPositionLimit;
  upper_limits_ = model_.upperPositionLimit;
  tau_buffer_ = Eigen::VectorXd::Zero(kDof);
  partial_dq_ = Eigen::MatrixXd::Zero(kDof, kDof);
  partial_dv_ = Eigen::MatrixXd::Zero(kDof, kDof);
  partial_dtau_ = Eigen::MatrixXd::Zero(kDof, kDof);
}

// The payload is merged into the inertia of the last link, so every
// algorithm below sees the arm plus the workpiece.
void RobotModel::setPayload(const Payload& payload) {
  payload_ = payload;
  if (payload.mass > 0.0) {
    model_.inertias[tip_] =
        tip_inertia_ + pinocchio::Inertia(payload.mass, payload.com, payload.inertia);
  } else {
    model_.inertias[tip_] = tip_inertia_;
  }
}

Vec7 RobotModel::gravity(const Vec7& q) {
  return pinocchio::computeGeneralizedGravity(model_, data_, q);
}

void RobotModel::gravityDerivative(const Vec7& q, Mat7& dg_dq) {
  pinocchio::computeGeneralizedGravityDerivatives(model_, data_, q, partial_dq_);
  dg_dq = partial_dq_;
}

Vec7 RobotModel::forwardDynamics(const Vec7& q, const Vec7& v, const Vec7& tau) {
  return pinocchio::aba(model_, data_, q, v, tau);
}

void RobotModel::forwardDynamicsDerivatives(const Vec7& q, const Vec7& v, const Vec7& tau,
                                            Mat7& da_dq, Mat7& da_dv, Mat7& minv) {
  pinocchio::computeABADerivatives(model_, data_, q, v, tau, partial_dq_, partial_dv_,
                                   partial_dtau_);
  da_dq = partial_dq_;
  da_dv = partial_dv_;
  minv = partial_dtau_;
  // Pinocchio fills only the upper triangle of the inverse mass matrix; mirror it.
  minv.triangularView<Eigen::StrictlyLower>() =
      minv.transpose().triangularView<Eigen::StrictlyLower>();
}

Vec7 RobotModel::inverseDynamics(const Vec7& q, const Vec7& v, const Vec7& a) {
  return pinocchio::rnea(model_, data_, q, v, a);
}

// CRBA also fills only the upper triangle; the copy below makes it symmetric.
Mat7 RobotModel::massMatrix(const Vec7& q) {
  pinocchio::crba(model_, data_, q);
  Mat7 m = data_.M;
  m.triangularView<Eigen::StrictlyLower>() = m.transpose().triangularView<Eigen::StrictlyLower>();
  return m;
}

}  // namespace iiwa_control_law
