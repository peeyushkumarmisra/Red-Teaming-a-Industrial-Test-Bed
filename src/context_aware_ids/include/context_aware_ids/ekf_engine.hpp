#ifndef CONTEXT_AWARE_IDS_EKF_ENGINE_HPP_
#define CONTEXT_AWARE_IDS_EKF_ENGINE_HPP_

#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <string>

#include "context_aware_ids/kinematics_engine.hpp"

namespace context_aware_ids
{

template <size_t DOF>
class EKFEngine
{
public:
  static constexpr size_t STATE_SIZE = DOF * 2; // N joints has 2*N state variables

  using StateVector = Eigen::Matrix <double, STATE_SIZE, 1>;
  using StateMatrix = Eigen::Matrix <double, STATE_SIZE, STATE_SIZE>;
  using MeasurementVector = Eigen::Matrix <double, DOF, 1>;
  using ObservationMatrix = Eigen::Matrix <double, DOF, STATE_SIZE>;
  using MeasurementNoiseMatrix = Eigen::Matrix <double, DOF, DOF>;
  using JointVector = Eigen::Matrix <double, DOF, 1>;
  
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
    // Initialize the physics engine with URDF on the heap
    physics_engine_ = std::make_unique<KinematicsEngine<DOF>>(urdf_path);
  }



  // [[nodiscard]] warns if the residual isn't used
  // 'inline' suggests the compiler should paste this code directly where it's called
  [[nodiscard]] inline MeasurementVector compute_residual
  (
    const MeasurementVector& z_actual,
    const JointVector& incoming_torques,
    int payload_context
  )
  {
    const double dt = 0.001; // 1000 Hz loop time
    physics_engine_->apply_payload_context(payload_context); // Weather it is loaded or not
    tj_ = physics_engine_->calc_true_jacobian(isp_, incoming_torques, dt); // Calculates true jacobian
    isp_ = tj_ * isp_;   // Predicting the state head using the physical kinematic model
    ic_ = (tj_ * ic_ * tj_.transpose()) + pm_;   // Error Covariance
    MeasurementVector z_pred = om_ * isp_;    // Expected Sensor Measurements
    MeasurementVector residual = z_actual - z_pred;   // Innovation Residual
    
    MeasurementNoiseMatrix S = (om_ * ic_ * om_.transpose()) + mn_;   // Innovation Covariance (S)
    Eigen::Matrix<double, STATE_SIZE, DOF> K = ic_ * om_.transpose() *S.inverse();   // Kalman Gain (K)
    isp_ = isp_ + (K*residual);   // Correct the internal state
    StateMatrix I = StateMatrix::Identity();
    ic_ = (I - (K*om_)) * ic_;   // Correct the covariance
    
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