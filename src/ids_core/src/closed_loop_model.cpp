#include "ids_core/closed_loop_model.hpp"

namespace ids_core {

using iiwa_control_law::computeTorque;
using iiwa_control_law::ReferencePoint;
using iiwa_control_law::referenceAt;
using iiwa_control_law::ReferenceSample;

ClosedLoopModel::ClosedLoopModel(iiwa_control_law::RobotModel& model, const ModelConfig& config,
                                 InputSource source)
    : model_(model), config_(config), source_(source) {
  da_dq_.setZero();
  da_dv_.setZero();
  minv_.setZero();
  dg_dq_.setZero();
  step_.setZero();
  step_input_.setZero();
}

Vec7 ClosedLoopModel::nominalTorque(const Vec7& q, const Vec7& v, const ReferenceSample& sample,
                                    double t) {
  const ReferencePoint r = referenceAt(sample, t, config_.reference_hold);
  return computeTorque(config_.gains, q, v, r, model_.gravity(q));
}

void ClosedLoopModel::propagate(State& x, double t0, int steps, const ReferenceTrack& track,
                                const Vec7& applied_torque, Mat21* f, Mat21x7* g,
                                const Vec7* disturbance) {
  const double h = config_.controller_period;
  const bool jacobians = f != nullptr && g != nullptr;
  if (jacobians) {
    f->setIdentity();
    g->setZero();
  }
  const Mat7 eye = Mat7::Identity();
  for (int k = 0; k < steps; ++k) {
    const double t = t0 + k * h;
    Vec7 tau;
    Mask7 saturated = Mask7::Constant(false);
    if (source_ == InputSource::kTrustedReference) {
      const ReferenceSample* sample = track.at(t, config_.reference_latency);
      const ReferencePoint r = sample != nullptr
                                   ? referenceAt(*sample, t, config_.reference_hold)
                                   : ReferencePoint{x.q, Vec7::Zero()};
      tau = computeTorque(config_.gains, x.q, x.v, r, model_.gravity(x.q), &saturated);
    } else {
      tau = applied_torque;
    }
    Vec7 u = tau - x.b.cwiseProduct(x.v);
    if (disturbance != nullptr) {
      u += *disturbance;
    }
    const Vec7 a = model_.forwardDynamics(x.q, x.v, u);
    if (jacobians) {
      model_.forwardDynamicsDerivatives(x.q, x.v, u, da_dq_, da_dv_, minv_);
      Mat7 du_dq = Mat7::Zero();
      Mat7 du_dv = Mat7::Zero();
      du_dv.diagonal() = -x.b;
      if (source_ == InputSource::kTrustedReference) {
        model_.gravityDerivative(x.q, dg_dq_);
        Mat7 ctrl_q = dg_dq_;
        ctrl_q.diagonal() -= config_.gains.kp;
        for (int i = 0; i < kDof; ++i) {
          if (!saturated(i)) {
            du_dq.row(i) = ctrl_q.row(i);
            du_dv(i, i) -= config_.gains.kd(i);
          }
        }
      }
      const Mat7 a_q = da_dq_ + minv_ * du_dq;
      const Mat7 a_v = da_dv_ + minv_ * du_dv;
      const Mat7 a_b = -(minv_ * x.v.asDiagonal());
      const Mat7 dv_dq = h * a_q;
      const Mat7 dv_dv = eye + h * a_v;
      const Mat7 dv_db = h * a_b;
      const Mat7 dv_dd = h * minv_;
      step_.setZero();
      step_.block<kDof, kDof>(0, 0) = eye + h * dv_dq;
      step_.block<kDof, kDof>(0, kDof) = h * dv_dv;
      step_.block<kDof, kDof>(0, 2 * kDof) = h * dv_db;
      step_.block<kDof, kDof>(kDof, 0) = dv_dq;
      step_.block<kDof, kDof>(kDof, kDof) = dv_dv;
      step_.block<kDof, kDof>(kDof, 2 * kDof) = dv_db;
      step_.block<kDof, kDof>(2 * kDof, 2 * kDof) = eye;
      step_input_.setZero();
      step_input_.block<kDof, kDof>(0, 0) = h * dv_dd;
      step_input_.block<kDof, kDof>(kDof, 0) = dv_dd;
      *g = step_ * (*g) + step_input_;
      *f = step_ * (*f);
    }
    x.v += h * a;
    x.q += h * x.v;
  }
}

}  // namespace ids_core