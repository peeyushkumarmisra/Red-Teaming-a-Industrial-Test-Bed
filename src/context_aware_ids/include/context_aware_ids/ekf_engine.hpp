#ifndef CONTEXT_AWARE_IDS_EKF_ENGINE_HPP_
#define CONTEXT_AWARE_IDS_EKF_ENGINE_HPP_

#include <cmath>
#include <memory>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Cholesky>

#include "context_aware_ids/kinematics_engine.hpp"

namespace context_aware_ids
{

template <size_t DOF>
class EKFEngine
{
public:
  static constexpr size_t STATE_SIZE = DOF * 2; // N joints has 2*N state variables

  using StateVector       = Eigen::Matrix <double, STATE_SIZE, 1>;
  using StateMatrix       = Eigen::Matrix <double, STATE_SIZE, STATE_SIZE>;
  using MeasurementVector = Eigen::Matrix <double, DOF, 1>;
  using ObservationMatrix = Eigen::Matrix <double, DOF, STATE_SIZE>;
  using MeasurementNoiseMatrix = Eigen::Matrix <double, DOF, DOF>;
  using JointVector       = Eigen::Matrix <double, DOF, 1>;
  
  // 'explicit' prevents C++ from accidentally converting strings into EKFEngines
  explicit EKFEngine(const std::string& urdf_path) // Constructor to avoid runtime overhead
  { 
    isp_.setZero();      // Initial state prediction is zero
    ic_.setIdentity();   // Initial confidence (Error Covariance)
    pm_.setIdentity();   // Process noise covariance
    mn_.setIdentity();   // Measurement noise covariance
    om_.setZero();       // Observation matrix
    tj_.setIdentity();   // State Transition Jacobian as a baseline
    // The observation matrix OM simply extracts the first N position states 
    // from the 2*N state vector to compare against sensor readings
    for(size_t i = 0; i < DOF; ++i)
    {
        om_(i, i) = 1.0; 
    }
    // Tuned for physical units
    pm_.diagonal().head(DOF).setConstant(1e-4);  // position process noise
    pm_.diagonal().tail(DOF).setConstant(1e-2);  // velocity process noise
    mn_.diagonal().setConstant(1e-6);            // encoder measurement noise
    // Initialize the physics engine with URDF on the heap
    physics_engine_ = std::make_unique<KinematicsEngine<DOF>>(urdf_path);
  }

  // [[nodiscard]] warns if the residual isn't used
  // 'inline' suggests the compiler should paste this code directly where it's called
  [[nodiscard]] inline MeasurementVector compute_residual
  (
    const MeasurementVector& incoming_positions,
    const JointVector& control_inputs,
    int payload_context,
    double dt
  )
  {
    // Prediction Phase
    physics_engine_->apply_payload_context(payload_context);
    tj_   = physics_engine_->calc_jacobian(isp_, control_inputs, dt);
    isp_  = physics_engine_->predict_state(isp_, control_inputs, dt); // Predict state
    ic_   = (tj_ * ic_ * tj_.transpose()) + pm_;  // Predict covariance
    // Innovation Phase
    MeasurementVector z_pred = om_ * isp_;
    MeasurementVector residual = incoming_positions - z_pred;
    // Update Phase
    Eigen::Matrix<double, DOF, DOF> S = (om_ * ic_ * om_.transpose()) + mn_;
    auto ldlt = S.ldlt();
    if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) {return residual;} // To prevent NaN
    Eigen::Matrix<double, STATE_SIZE, DOF> k = 
      ic_ * om_.transpose() * ldlt.solve(Eigen::Matrix<double, DOF, DOF>::Identity());
    isp_ = isp_ + (k * residual);
    StateMatrix I_KH = StateMatrix::Identity() - (k * om_);
    ic_ = (I_KH * ic_ * I_KH.transpose()) + (k * mn_ * k.transpose());
    ic_ = 0.5 * (ic_ + ic_.transpose());
    return residual;
  }

private:
  StateVector isp_;             // State estimate [q, q_dot]
  StateMatrix ic_;              // Error covariance matrix
  StateMatrix tj_;              // State Transition Jacobian
  StateMatrix pm_;              // Process noise (unmodeled physical dynamics)
  ObservationMatrix om_;        // Maps state vector to sensor measurements (Observation Matrix)
  MeasurementNoiseMatrix mn_;   // Sensor noise (Gaussian chatter)

  std::unique_ptr<KinematicsEngine<DOF>> physics_engine_;    // Unique pointer for Physics Class
};

} // namespace context_aware_ids

#endif // CONTEXT_AWARE_IDS_EKF_ENGINE_HPP_